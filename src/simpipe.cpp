//
//  simpipe.cpp
//
//  Created by Enrico Eberhard
//
//	SimPipe simulates passive and active dynamics,
//  and pipes the resulting state information via
//  raw ethernet packets
//
//	This runs a forward dynamic simulation of a mujoco
//	model in real-time. The position and velocity of
//	a chosen interface point on the model (referred to
//	as the Portal) is piped over ethernet to the robot
//	controller watchfrog. As the robot moves the end-effector
//	to the target, the force sensor picks up real-world
//	ground reaction force, and pipes this back here.
//	The real force is applied to the portal in simulation,
//	and so the virtual mechanism experiences real contact dynamics.


#include "mujocoToolbox.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "getopt.h"
#include "ethraw.h"
#include "sys/resource.h"

#define BILLION 1000000000

mjModel* m = 0;
mjData* d = 0;

int eth_device;

bool timing = false;
bool loginfo = false;
char logfile[100];

char modelfile[100];

bool rendered = false;

struct STATE {
    mjtNum x[3];
    mjtNum v[3];
};

enum RUN_MODE {
    SEND,
    RECEIVE
} run_mode = SEND;


STATE state;


/* Function Prototypes */

// step through forward dynamics until simtime catches up
//	to real time
void real_time_step(mjModel* m, mjData* d);

// get a packet from watchfrog containing ground forces and
//  current (measured) state of robot
void read_from_watchfrog(unsigned char* data);

// write a target state to robot
void write_to_watchfrog(mjModel* m, mjData* d);

// get the state of the interface point in the dynamic simulation
void getPortalState(mjModel* m, mjData* d, struct STATE* state);

// apply forces to the interface point in the dynamic simulation
void applyPortalForce(mjModel* m, mjData* d, struct FT ft);

// parse input arguments
int parseArgsLong(int argc, const char **argv);

//utility to measure time difference between timespec structures
double timespec_diff(struct timespec *start, struct timespec *end);

//butterworth filter with 10% corner frequencies
void lp_filter_4thOrder_10percent(double* sig, int size);

//basic callback for debugging receive mode.
void print_data_callback(unsigned char* data)



/* Function Definitions */


//program entry point
int main(int argc, const char** argv)
{
    
    printf("+---------+\n| SIMPIPE |\n+---------+\n");
    
    //set defaults
    strncpy(modelfile,"../models/planardemo.xml", 100);
    
    set_eth_defaults("78:24:af:8b:6b:24",
                     "eno1",
                     0x1234);
    
    parseArgsLong(argc, argv);
    
    eth_device = initialize_eth_device();
    
    //make this process highest priority!
    int which = PRIO_PROCESS;
    id_t pid;
    int priority = -20;
    
    pid = getpid();
    printf("Setting process %i priority to %i - returned %i\n",
           pid, priority, setpriority(which, pid, priority));
    
    if (run_mode==RECEIVE) {
        //Debug mode waits to receive packets and prints them
        while(1) {
            read_frames(eth_device, print_data_callback);
        }
    }
    
    
    //activate mujoco
    checkAndActivate();
    
    //initialize GLFW and window
    GLFWwindow* window = glfwInitWindow();
    if (!window) {
        return 1;
    }
    
    //load model
    loadmodel(window, modelfile, 0);
    
    printf("Starting simulation loop\n");
    
    //if not rendering window, just run the simulation
    if(!rendered) {
        
        while(1) {
            real_time_step(m,d);
        }
        
        return 0;
    }
    
    //if rendering window...
    paused = false;
    
    while(!glfwWindowShouldClose(window))
    {
        if(!paused) {
            mjtNum factor = (slowmotion ? 10 : 1);
            
            // advance effective simulation time by 1/refreshrate
            mjtNum startsimtm = d->time;
            while((d->time-startsimtm)*factor < 1.0f / refreshrate) {
                
                //apply perturbations
                mju_zero(d->xfrc_applied, 6 * m->nbody);
                if (pert.select > 0)
                    mjv_applyPerturbForce(m, d, &pert);
                
                real_time_step(m,d);
            }
        }
        else {
            // apply pose perturbations, run mj_forward
            mjv_applyPerturbPose(m, d, &pert, 1);
            mj_forward(m, d);
        }
        
        // update the window
        render(window);
        
        // handle events (this calls all callbacks)
        glfwPollEvents();
        
    }
    // delete everything we allocated
    closeAndTerminate();
    
    return 0;
    
}


void real_time_step(mjModel* m, mjData* d)
{
    
#define PRINT_HZ 5000
    
    static struct timespec start, last;
    struct timespec now;
    static const int printrate = (int)((double)PRINT_HZ);
    static int simsteps, iterations;
    double realtime, lasttime;
    
    struct timespec tic;
    
    
    read_frames(eth_device, read_from_watchfrog);
    
    //on first run, set now as the start of sim time
    if (start.tv_sec == 0) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        clock_gettime(CLOCK_MONOTONIC, &last);
    }
    
    
    //find real time elasped since sim begin
    clock_gettime(CLOCK_MONOTONIC, &tic);
    realtime = timespec_diff(&start, &tic);
    
    //regulate step time so physics is real-time
    while (d->time < realtime) {
        mj_step(m,d);
        simsteps++;
    }
    
    //send the updated state to the robot
    write_to_watchfrog(m,d);
    
    //log information at PRINT_HZ
    iterations++;
    if (iterations%printrate == 0) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        lasttime = timespec_diff(&last, &now);
        
        printf("%6.3fs (RTF %2.3f): %6.1fHz, %4i steps - %i\n",
               d->time,  d->time/realtime,
               (double)printrate/lasttime,
               simsteps, iterations);
        
        simsteps = 0;
        clock_gettime(CLOCK_MONOTONIC, &last);
    }
    
}


void getPortalState(mjModel* m, mjData* d, struct STATE* state)
{
    
    static const int portalp = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalp")];
    static const int portalx = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalx")];
    static const int portalv = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalv")];
    static const int portalw = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalw")];
    
    
    mju_copy3(state->x, &d->sensordata[portalp]);
    
    //y rotation = atan2(x,z)
    state->x[1] = atan2(d->sensordata[portalx], d->sensordata[portalx + 2]) - M_PI_2;
    
    mju_copy3(state->v, &d->sensordata[portalv]);
    state->v[1] = d->sensordata[portalw + 1];
}

void applyPortalForce(mjModel* m, mjData* d, mjtNum* ft)
{
    
    static const int portalfx = mj_name2id(m, mjOBJ_ACTUATOR, "portalfx");
    static const int portalfy = mj_name2id(m, mjOBJ_ACTUATOR, "portalfy");
    static const int portalfz = mj_name2id(m, mjOBJ_ACTUATOR, "portalfz");
    
    d->ctrl[portalfx] = ft[0];
    d->ctrl[portalfy] = ft[1];
    d->ctrl[portalfz] = ft[2];
}


void write_to_watchfrog(mjModel* m, mjData* d)
{
    
    static const char* format =
    "BOT: %+5.2f, %+5.4f, %+5.2f, %+5.4f, %+5.4f, %+5.4f";
    
    char data[ETHER_PAYLOAD_LEN];
    
    //get position, orientation and velocities of interface
    getPortalState(m, d, &state);
    
    //generate data string with state
    sprintf(data, format,
            state.x[0] * 1000, state.x[1], state.x[2] * 1000,
            state.v[0] * 1000, state.v[1], state.v[2] * 1000);
    
    send_frame(eth_device, (unsigned char *)data, (size_t)strlen((const char*)data));
}


void read_from_watchfrog(unsigned char* data)
{
    
    long int s[2];
    mjtNum x[3];
    mjtNum f[3];
    
    mjtNum force[3];
    
    static const char* format =
    "FROG: %li, %li, %lf, %lf, %lf, %lf, %lf, %lf";
    
    if(8==sscanf((const char*)data, format,
                 &s[0], &s[1],
                 &f[0], &f[1], &f[2],
                 &x[0], &x[1], &x[2])) {
        mju_copy3(force, f);
    }
    else {
        mju_zero3(force);
    }
    
    //apply forces
    applyPortalForce(m, d, force);
    
    static FILE* log;
    
    if(loginfo) {
        
        if(!log) {
            log = fopen(logfile, "w");
        }
        
        fprintf(log,
                "%6li, %6li, "
                "%+7.6f, %+7.6f, %+7.6f, "
                "%+5.2f, %+5.4f, %+5.2f, "
                "%+5.2f, %+5.4f, %+5.2f\n",
                s[0], s[1],
                f[0], f[1], f[2],
                x[0], x[1], x[2],
                state.x[0]*1000, state.x[1], state.x[2]*1000);
    }
}





const char help_str[] =
"SIMPIPE HELP\n"
"\t A program for simulating and piping state information over ethernet\n"
""
"Usage:\n"
"\t sudo ./simpipe [-opt] [arg] [[-opt2] [arg2] ...]\n"
"Options:\n"
"\t -m | --model {filename}            \t filename of model xml\n"
"\t -d | --dest  {xx:xx:xx:xx:xx:xx}   \t destination MAC address\n"
"\t -i | --iface {interface e.g. eth0} \t network interface\n"
"\t -p | --proto {protocol 0x0800}     \t 2byte protocol in hex format\n"
"\t -l | --log   {filename}            \t filename to log run-time info\n"
"\n"
"\t -h | --help         \t Shows options\n"
"\t -w | --hide         \t Doesn't render the simulation\n"
"\t -r | --receive      \t For debug only, sets receive mode\n"
"\t -s | --send         \t For debug only, sets send mode\n"
;


int parseArgsLong(int argc, const char **argv)
{
    
    static struct option long_options[] =
    {
        {"model",		required_argument, 0, 'm'},
        {"dest",		required_argument, 0, 'd'},
        {"iface",		required_argument, 0, 'i'},
        {"proto",		required_argument, 0, 'p'},
        {"log",			required_argument, 0, 'l'},
        {"help",		no_argument,       0, 'h'},
        {"hide",		no_argument,       0, 'w'},
        {"receive",		no_argument,	   0, 'r'},
        {"send",		no_argument,	   0, 's'},
        {0, 0, 0, 0}
    };
    
    static const char* short_options = "m:d:i:p:l:hwtrs";
    
    // getopt_long stores the option index here.
    int option_index = 0;
    
    int c;
    
    while ((c = getopt_long(argc,
                            (char**)argv,
                            short_options,
                            long_options,
                            &option_index)) != -1) {
        
        switch (c) {
            case 0:
                if (long_options[option_index].flag != 0)
                    break;
                printf ("option %s", long_options[option_index].name);
                if (optarg)
                    printf (" with arg %s", optarg);
                printf ("\n");
                break;
                
            case 'm':
                strncpy(modelfile, optarg, 100);
                printf("-> loading model %s\n", modelfile);
                break;
                
            case 'd':
                strncpy(dest_mac_str, optarg, strlen(optarg));
                
                break;
                
            case 'i':
                strncpy(interface, optarg, IFNAMSIZ);
                break;
                
            case 'p':
                sscanf(optarg, "0x%4hx", &protocol);
                break;
                
            case 'l':
                loginfo = true;
                strncpy(logfile, optarg, 100);
                printf("-> logging info to %s\n", logfile);
                break;
                
            case 'h':
                printf(help_str);
                exit(0);
                break;
                
            case 'r':
                run_mode = RECEIVE;
                break;
                
                
            case 't':
                timing = true;
                break;
                
            case 's':
                run_mode = SEND;
                break;
                
            case 'w':
                printf("-> rendering simulation\n");
                rendered = true;
                break;
                
            case '?':
                // getopt_long already printed an error message.
                printf("Use -h or --help for options\n");
                break;
                
            default:
                abort ();
        }
    }
    
    
    // Print any remaining command line arguments (non-options).
    if (optind < argc) {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        putchar ('\n');
    }
    
    printf("\n");
    
    return 0;
    
}


double timespec_diff(struct timespec *start, struct timespec *end)
{
    struct timespec diff;
    
    if((end->tv_nsec - start->tv_nsec) < 0) {
        diff.tv_sec = end->tv_sec - start->tv_sec - 1;
        diff.tv_nsec = end->tv_nsec - start->tv_nsec + BILLION;
    }
    else {
        diff.tv_sec = end->tv_sec - start->tv_sec;
        diff.tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    
    
    return (double) ((long double)diff.tv_sec +
                     ((long double)diff.tv_nsec / (long double)BILLION));
    
}

// 4th order Bessel low-pass filter
// Corner frequency 10% of sample freq (e.g. @1kHz, 100Hz is -3dB)
void lp_filter_4thOrder_10percent(double* sig, int size)
{
    
    static const double GAIN = 6.893641214e+01;
    
    static const double c[] = {
        -0.0503462932,
        0.3599070274,
        -1.0458620167,
        1.5042033315
    };
    
    static float xv[4+1][100], yv[4+1][100];
    
    for (int ch=0; ch < size; ch++) {
        
        //shift inputs in time
        xv[0][ch] = xv[1][ch]; xv[1][ch] = xv[2][ch]; xv[2][ch] = xv[3][ch]; xv[3][ch] = xv[4][ch];
        xv[4][ch] = sig[ch] / GAIN;
        
        //shift previous outputs
        yv[0][ch] = yv[1][ch]; yv[1][ch] = yv[2][ch]; yv[2][ch] = yv[3][ch]; yv[3][ch] = yv[4][ch];
        
        //calculate weighted output
        yv[4][ch] =   (xv[0][ch] + xv[4][ch]) + 4 * (xv[1][ch] + xv[3][ch]) + 6 * xv[2][ch]
        + (c[0] * yv[0][ch]) + (c[1] * yv[1][ch])
        + (c[2] * yv[2][ch]) + (c[3] * yv[3][ch]);
        
        sig[ch] = yv[4][ch];
    }
}


void print_data_callback(unsigned char* data)
{
    printf("%s\n", data);
}
