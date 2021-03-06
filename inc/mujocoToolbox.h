//
//  mujocoToolbox.h
//
//
//  Created by Enrico Eberhard on 03/02/2017.
//
//  This header provides simplified access to some common
//  routines when using mujoco for simulation, and in
//  particular for rendering using the GLFW library.
//
//
//

//-----------------------------------//
//  Parts of this file by MuJoCo.    //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#ifndef mujocoToolbox_h
#define mujocoToolbox_h


#ifdef _DEBUG_
#undef _DEBUG_
#endif
//uncomment to enable debug mode
//	(runs test code and is more verbose)
//#define _DEBUG_ 1


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

#define GLFW_WIN_WIDTH	1200
#define GLFW_WIN_HEIGHT	900

// model and data structs from main .c
extern mjModel* m;
extern mjData* d;


// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;

// OpenGL rendering
int refreshrate;
const int fontscale = mjFONTSCALE_150;
mjrContext con;
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

//states
bool paused = false;
bool slowmotion = false;
bool showinfo = true;
bool showoption = false;
bool showfullscreen = false;
int showhelp = 1;

//strings
char opt_title[1000] = "";
char opt_content[1000];
char lastfile[1000] = "";   //save loaded filename for quick ctrl+l reload
char status[1000] = "";

// for mouse functions
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// help strings
const char help_title[] =
"Help\n"
"Option\n"
"Info\n"
"Depth\n"
"Full screen\n"
"Stereo\n"
"Profiler\n"
"Slow motion\n"
"Key reset\n"
"Pause\n"
"Reset\n"
"Forward\n"
"Back\n"
"Forward 100\n"
"Back 100\n"
"Autoscale\n"
"Reload\n"
"Geoms\n"
"Sites\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Camera\n"
"Frame\n"
"Label\n"
"Fontsize";


const char help_content[] =
"F1\n"
"F2\n"
"F3\n"
"F4\n"
"F5\n"
"F6\n"
"F7\n"
"Enter\n"
"Page Up/Down\n"
"Space\n"
"BackSpace\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Ctrl A\n"
"Ctrl L\n"
"0 - 4\n"
"Shift 0 - 4\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"[ ]\n"
"; '\n"
". /\n"
"- =";



/* Public Function Prototypes */


void render(GLFWwindow* window);

GLFWwindow* glfwInitWindow(void);
void closeAndTerminate(void);
void checkAndActivate(const char* filename);
void makeoptionstring(const char* name, char key, char* buf);
void autoscale(GLFWwindow* window);
void loadmodel(GLFWwindow* window, const char* filename, const char* xmlstring);

//callback functions
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void drop(GLFWwindow* window, int count, const char** paths);


//file reading utility
void sizeofCSV(int* rows, int* cols, FILE* file);
void CSV2qpos(const mjModel* m, mjtNum* q, int rows, int cols, FILE* file);

//kinematics
void qpos2qvel(const mjModel* m, mjtNum* q, mjtNum* v, int t, mjtNum dt);
void qvel2qacc(const mjModel* m, mjtNum* v, mjtNum* a, int len);
int setInitialPoseFromFile(const mjModel* m, const char* filename);
int getPoseFromFile(const mjModel* m, const char* filename, mjtNum* q, int line);

//other
void waitSeconds(double delay);

//Extra callbacks
typedef void (*keyCB)(int, int);
bool extraCB = false;
keyCB extraKeyCB;

//use this to add custom functionality to keys on a per-program basis
void glfwSetExtraKeyCallback(keyCB cbFun){
    extraKeyCB = cbFun; extraCB = true;
}


typedef void (*renderFun)(GLFWwindow*);
bool extraRender = false;
renderFun extraRenderFun;

//use this to add a custom function to the normal render routine
// (for instance for updating a figure)
void glfwSetExtraRenderFun(renderFun rFun){
    extraRenderFun = rFun; extraRender = true;
}

void glfwUnsetExtraRenderFun(){
    extraRender = false;
}



/* Public Function Definitions */


// this main render function displays the current model in the window
void render(GLFWwindow* window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;
    
    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);
    mjrRect smallrect = rect;
    
    
    // no model: empty screen
    if (!m)
    {
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "There was a problem with the model file", 0, &con);
        
        // swap buffers
        glfwSwapBuffers(window);
        return;
    }
    
    if (!paused) {
        
        // camera string
        char camstr[20];
        if( cam.type==mjCAMERA_FREE )
            strcpy(camstr, "Free");
        else if( cam.type==mjCAMERA_TRACKING )
            strcpy(camstr, "Tracking");
        else
            sprintf(camstr, "Fixed %d", cam.fixedcamid);
        
        // solver error
        mjtNum solerr = 0;
        if (d->solver_iter) {
            int ind = mjMIN(d->solver_iter-1,mjNSOLVER-1);
            solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
            if( solerr==0 )
                solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
        }
        solerr = mju_log10(mju_max(mjMINVAL, solerr));
        
        // status
        sprintf(status, "%-20.3f\n%d  (%d con)\n%.3f\n%.0f\n%.2f\n%.1f  (%d it)\n%.1f %.1f\n%s\n%s\n%s",
                d->time,
                d->nefc,
                d->ncon,
                d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number),
                1.0/(glfwGetTime()-lastrendertm),
                d->energy[0]+d->energy[1],
                solerr,
                d->solver_iter,
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[1])),
                camstr,
                mjFRAMESTRING[vopt.frame],
                mjLABELSTRING[vopt.label]
                );
    }
    lastrendertm = glfwGetTime();
    
    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);
    
    // render
    mjr_render(rect, &scn, &con);
    
    char timestr[30];
    sprintf(timestr,"\n%-20.3f",d->time);
    
    // show overlays
    if (showhelp==1)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
    else if (showhelp==2)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);
    
    // show info
    if (showinfo) {
        if(paused)
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, "PAUSED\nTime", timestr, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
                        "Time\nSize\nCPU\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nLabel", status, &con);
    }
    
    // show options
    if (showoption) {
        int i;
        char buf[100];
        
        // fill titles on first pass
        if (!opt_title[0]) {
            for (i = 0; i < mjNRNDFLAG; i++) {
                makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                strcat(opt_title, "\n");
            }
            for (i = 0; i < mjNVISFLAG; i++) {
                makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                if (i < mjNVISFLAG - 1)
                    strcat(opt_title, "\n");
            }
        }
        
        // fill content
        opt_content[0] = 0;
        for (i = 0; i < mjNRNDFLAG; i++) {
            strcat(opt_content, scn.flags[i] ? " + " : "   ");
            strcat(opt_content, "\n");
        }
        for (i = 0; i < mjNVISFLAG; i++) {
            strcat(opt_content, vopt.flags[i] ? " + " : "   ");
            if (i < mjNVISFLAG - 1)
                strcat(opt_content, "\n");
        }
        
        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, opt_title, opt_content, &con);
    }
    
    
    //do any last render functions before swapping buffers
    if (extraRender)
        extraRenderFun(window);
    
    
    // swap buffers
    glfwSwapBuffers(window);
}


GLFWwindow* glfwInitWindow(void) {
    
    // init GLFW
    if (!glfwInit())
        return 0;
    
    // get refreshrate
    refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
    
    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);
    
    // try stereo if refresh rate is at least 100Hz
    GLFWwindow* window = 0;
    if (refreshrate>=100) {
        glfwWindowHint(GLFW_STEREO, 1);
        window = glfwCreateWindow(GLFW_WIN_WIDTH, GLFW_WIN_HEIGHT, "Simulate", NULL, NULL);
    }
    
    // no stereo: try mono
    if (!window) {
        glfwWindowHint(GLFW_STEREO, 0);
        window = glfwCreateWindow(GLFW_WIN_WIDTH, GLFW_WIN_HEIGHT, "Simulate", NULL, NULL);
    }
    
    if (!window) {
        glfwTerminate();
        return 0;
    }
    
    // make context current, request v-sync on swapbuffers
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    // save window-to-framebuffer pixel scaling (needed for OSX scaling)
    int width, width1, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width1, &height);
    window2buffer = (double)width1 / (double)width;
    
    
    // init MuJoCo rendering, get OpenGL info
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, fontscale);
    
    
    // set GLFW callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetDropCallback(window, drop);
    glfwSetWindowRefreshCallback(window, render);
    
    return window;
}



void closeAndTerminate(void){
    
    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    
    // terminate
    glfwTerminate();
    mj_deactivate();
}

void checkAndActivate(const char* filename = "mjkey.txt")
{
    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
    if (mjVERSION_HEADER!=mj_version())
        mju_error("Headers and library have different versions");
    
    mj_activate(filename);
}


// make option string
void makeoptionstring(const char* name, char key, char* buf)
{
    int i = 0;
    int cnt = 0;
    
    // copy non-& characters
    while (name[i] && i<50 ) {
        if (name[i] != '&') {
            buf[cnt++] = name[i];
        }
        i++;
    }
    
    // finish
    buf[cnt] = ' ';
    buf[cnt+1] = '(';
    buf[cnt+2] = key;
    buf[cnt+3] = ')';
    buf[cnt+4] = 0;
}


// center and scale view
void autoscale(GLFWwindow* window)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
    
    // set to free camera
    cam.type = mjCAMERA_FREE;
}


// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename, const char* xmlstring)
{
    // make sure one source is given
    if( !filename && !xmlstring )
        return;
    
    // load and compile
    char error[1000] = "could not load binary model";
    mjModel* mnew = 0;
    
    if (strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb")) {
        mnew = mj_loadModel(filename, 0);
    }
    else {
        mnew = mj_loadXML(filename, 0, error, 1000);
    }
    
    if (!mnew) {
        printf("%s\n", error);
        return;
    }
    
    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
    mj_forward(m, d);
    
    // save filename for reload
    if (!xmlstring)
        strcpy(lastfile, filename);
    else
        lastfile[0] = 0;
    
    // re-create custom context
    mjr_makeContext(m, &con, fontscale);
    
    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    
    
    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);
    
    // set window title to mode name
    if (window && m->names)
        glfwSetWindowTitle(window, m->names);
}

// version of loadmodel that doesn't return a window
void loadmodel_no_win(const char* filename, const char* xmlstring)
{
    // make sure one source is given
    if (!filename && !xmlstring)
        return;
    
    // load and compile
    char error[1000] = "could not load binary model";
    mjModel* mnew = 0;
    
    if (strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb")) {
        mnew = mj_loadModel(filename, 0);
    }
    else {
        mnew = mj_loadXML(file name, 0, error, 1000);
    }
    
    if (!mnew) {
        printf("%s\n", error);
        return;
    }
    
    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
    mj_forward(m, d);
}


//starter shortcut
GLFWwindow* startMuJoCo(const char* filename) {
    
    //activate mujoco
    checkAndActivate();
    
    //initialize GLFW and window
    GLFWwindow* window = glfwInitWindow();
    if (!window)
        return NULL;
    
    //load model
    loadmodel(window,filename, 0);
    
    return window;
}


//-------------------------------- callback functions ------------------------------------

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    
    // require model
    if (!m)
        return;
    
    
    // if defined, execute custom key callbacks
    if (extraCB) {
        (*extraKeyCB)(key, act);
    }
    
    
    // do not act on release
    if (act == GLFW_RELEASE)
        return;
    
    
    //keycodes defined in glfw3.h
    switch (key) {
            
        case GLFW_KEY_F2:                   // glfw display options
            showoption = !showoption;
            break;
            
        case GLFW_KEY_F5:                   // toggle full screen
            showfullscreen = !showfullscreen;
            if (showfullscreen)
                glfwMaximizeWindow(window);
            else
                glfwRestoreWindow(window);
            break;
            
            
        case GLFW_KEY_SPACE:               // pause
            paused = !paused;
            break;
            
            
        case GLFW_KEY_BACKSPACE:            // reset
            autoscale(window);
            mj_resetData(m, d);
            mj_forward(m, d);
            break;
            
        case GLFW_KEY_ENTER:               // pause
            slowmotion = !slowmotion;
            break;
            
            
        case GLFW_KEY_ESCAPE:               // free camera
            cam.type = mjCAMERA_FREE;
            break;
            
            
        case '[':                           // previous fixed camera or free
            if (m->ncam && cam.type == mjCAMERA_FIXED) {
                if (cam.fixedcamid > 0)
                    cam.fixedcamid--;
                else
                    cam.type = mjCAMERA_FREE;
            }
            break;
            
        case ']':                           // next fixed camera
            if (m->ncam) {
                if (cam.type != mjCAMERA_FIXED) {
                    cam.type = mjCAMERA_FIXED;
                    cam.fixedcamid = 0;
                }
                else if (cam.fixedcamid<m->ncam-1) {
                    cam.fixedcamid++;
                }
            }
            break;
            
            
        default:
            
            // control keys
            if (mods & GLFW_MOD_CONTROL) {
                if (key == GLFW_KEY_A)
                    autoscale(window);
                else if (key == GLFW_KEY_L && lastfile[0])
                    loadmodel(window, lastfile, 0);
                
                break;
            }
            
            // toggle visualization flag
            for (int i = 0; i < mjNVISFLAG; i++) {
                if (key == mjVISSTRING[i][2][0])
                    vopt.flags[i] = !vopt.flags[i];
            }
            
            // toggle rendering flag
            for (int i = 0; i < mjNRNDFLAG; i++) {
                if (key == mjRNDSTRING[i][2][0])
                    scn.flags[i] = !scn.flags[i];
            }
            
            // toggle geom/site group
            for (int i = 0; i < mjNGROUP; i++) {
                if (key == i+'0') {
                    if (mods & GLFW_MOD_SHIFT)
                        vopt.sitegroup[i] = !vopt.sitegroup[i];
                    else
                        vopt.geomgroup[i] = !vopt.geomgroup[i];
                }
            }
    }
}



// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;
    
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
    
    
    // Alt: swap left and right
    if ((mods & GLFW_MOD_ALT))
    {
        bool tmp = button_left;
        button_left = button_right;
        button_right = tmp;
        
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            button = GLFW_MOUSE_BUTTON_RIGHT;
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            button = GLFW_MOUSE_BUTTON_LEFT;
    }
    
    
    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
    
    // require model
    if (!m)
        return;
    
    // set perturbation
    int newperturb = 0;
    if (act == GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select > 0) {
        // right: translate;  left: rotate
        if (button_right)
            newperturb = mjPERT_TRANSLATE;
        else if (button_left)
            newperturb = mjPERT_ROTATE;
        
        // perturbation onset: reset reference
        if (newperturb && !pert.active)
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;
    
    
    // detect double-click (250 msec)
    if (act == GLFW_PRESS && glfwGetTime() - lastclicktm < 0.25 && button == lastbutton) {
        // determine selection mode
        int selmode;
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            selmode = 1;
        else if (mods & GLFW_MOD_CONTROL)
            selmode = 3;
        else
            selmode = 2;
        
        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        
        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];
        int selgeom = mjv_select(m, d, &vopt,
                                 (mjtNum)width / (mjtNum)height,
                                 (mjtNum)lastx / (mjtNum)width,
                                 (mjtNum)(height-lasty) / (mjtNum)height,
                                 &scn, selpnt);
        int selbody = (selgeom>=0 ? m->geom_bodyid[selgeom] : 0);
        
        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3) {
            // copy selpnt if geom clicked
            if (selgeom >= 0)
                mju_copy3(cam.lookat, selpnt);
            
            // switch to tracking camera
            if (selmode == 3 && selbody) {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }
        
        // set body selection
        else {
            if (selbody) {
                // record selection
                pert.select = selbody;
                
                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
            }
            else {
                pert.select = 0;
            }
        }
        
        // stop perturbation on select
        pert.active = 0;
    }
    
    // save info
    if (act == GLFW_PRESS) {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}


// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;
    
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    
    // require model
    if (!m)
        return;
    
    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    
    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
    
    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;
    
    // move perturb or camera
    if( pert.active )
        mjv_movePerturb(m, d, action, dx/height, dy/height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // require model
    if (!m)
        return;
    
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// drop
void drop(GLFWwindow* window, int count, const char** paths)
{
    // make sure list is non-empty
    if (count>0)
        loadmodel(window, paths[0], 0);
}






//-------------------------------- IO utility functions ------------------------------------


//returns the size of a csv text file in rows and columns
void sizeofCSV(int* rows, int* cols, FILE* file){
    
    int ch, r, c;
    
    ch = r = 0; c = 1;
    
    while (!feof(file)) {
        ch = fgetc(file);
        if((r == 0) && (ch == ','))
            c++;
        if(ch == '\n')
            r++;
    }
    
    //very important - return file pointer to start of file
    rewind(file);
    
    //write output
    *rows = r; *cols = c;
}

//reads a single line of a csv text file
#define LINE_BUF_SIZE 1024
int CSVLineData(mjtNum* data, FILE* fp, int cols) {
    
    char line[LINE_BUF_SIZE];
    
    int c, read;
    int offset, bytes;
    
    if (fgets(line, LINE_BUF_SIZE, fp) != NULL) {
        
        offset = 0; read = 0;
        for (c = 0; c < cols - 1; c++) {
            if (sscanf(line + offset, "%lf, %n", &data[c], &bytes)) {
                read++;
                offset += bytes;
            }
        }
        
        if (sscanf(line + offset, "%lf \n", &data[c])) {
            read++;
        }
        
        return read;
        
    }
    else
        return 0;
    
    
}


//writes a csv data file to a qpos array (num_lines x m->nq)
void CSV2qpos(const mjModel* m, mjtNum* q, int rows, int cols, FILE* file) {
    
    int r, c, ret; //ret value can be used for error checking
    
    if (cols > m->nq)
        cols = m->nq;
    
    //fill the arrays
    for (r = 0; r < rows; r++) {
        //get each value followed by a comma
        for (c = 0; c < cols - 1; c++)
        {
            ret = fscanf(file,"%lf,",&q[r*cols + c]);
        }
        
        if (r < rows-1)	//get last value followed by newline
            ret = fscanf(file,"%lf\n",&q[r*cols + c]);
        else			//get last value at end of file
            ret = fscanf(file,"%lf",  &q[r*cols + c]);
        
    }
    
    //close the file now that we are finished
    fclose(file);
}

//writes a csv data file to a qvel sized array (num_lines x m->nv)
void CSV2dof(const mjModel* m, mjtNum* u, int rows, int cols, FILE* file) {
    
    int r, c, ret; //ret value can be used for error checking
    
    //fill the arrays
    for (r = 0; r < rows; r++) {
        //get each value followed by a comma
        for(c = 0; c < cols - 1; c++)
        {
            ret = fscanf(file,"%lf,",&u[r*cols + c]);
        }
        
        if(r < rows-1)    //get last value followed by newline
            ret = fscanf(file,"%lf\n",&u[r*cols + c]);
        else            //get last value at end of file
            ret = fscanf(file,"%lf",  &u[r*cols + c]);
        
    }
    
    //close the file now that we are finished
    fclose(file);
}




//sets qpos0 from first line of CSV
int setInitialPoseFromFile(const mjModel* m, const char* filename)
{
    FILE* file = fopen(filename, "r");
    
    if (!file) {
        printf("Problem opening file for setting initial pose!\n");
        fclose(file);
        return 1;
    }
    
    int r, c;
    
    //get the number of columns
    sizeofCSV(&r, &c, file);
    
    //ignore additional column information
    if (c > m->nq)
        c = m->nq;
    
#ifdef _WIN32
    mjtNum q[100]; //windows is weird about compiling unkown array size
#else
    mjtNum q[m->nq];
#endif
    
    CSV2qpos(m, q, 1, c, file);
    
    mju_copy(m->qpos0, q, m->nq);
    
    return 0;
}


//reads a line from a CSV joint file and saves it to q
//expects q to be mjtNum[m->nq]
int getPoseFromFile(const mjModel* m, const char* filename, mjtNum* q, int line)
{
    
#ifdef _DEBUG_
    printf("Getting pose from line %i of \"%s\"\n",line,filename);
#endif
    
    if (line <= 0) {
        printf("getPoseFromFile: Line indexing starts at 1 (requested %i)\n", line);
        line = 1;
    }
    
    
    FILE* file = fopen(filename, "r");
    
    if (!file) {
        printf("getPoseFromFile: Problem opening file \"%s\"\n", filename);
        fclose(file);
        return 1;
    }
    
    int r, c;
    
    //get the number of columns
    sizeofCSV(&r, &c, file);
#ifdef _DEBUG_
    printf("Rows: %i, Cols: %i\n",r,c);
#endif
    
    if (r < line) {
        printf("getPoseFromFile: Not enough rows (%i) in file \"%s\" for requested line (%i)\n",
               r, filename, line);
        fclose(file);
        return 2;
    }
    
    //ignore additional column information
    if (c > m->nq)
        c = m->nq;
    
#ifdef _WIN32
    mjtNum qread[10000]; //windows is weird about compiling unkown array size (VLA)
#else
    mjtNum qread[line*c];
#endif
    
    //read up to the requested line
    CSV2qpos(m, qread, line, c, file);
    
    
    //copy just that line for output
    mju_copy(q,&qread[(line-1)*c],c);
    
#ifdef _DEBUG_
    printf("Pose read:\n");
    mju_printMat(q,1,m->nq);
#endif
    
    
    return 0;
}


//saves a memory array into a CSV text file!
//supply the array, number of rows and columns (integers),
//and some "filename.txt"
int saveAsCSV(mjtNum* buf, int nr, int nc, const char* filename) {
    
    FILE* file = fopen(filename, "w");
    if (!file) {
        printf("Could not open input file %s\n",filename);
        return -1;
    }
    
    int r, c;
    for(r=0;r<nr;r++) {
        for(c=0;c<nc-1;c++) {
            fprintf(file,"%1.6f,",buf[r*nc + c]);
        }
        fprintf(file,"%1.6f\n",buf[r*nc + c]);
    }
    
    fclose(file);
    
    return 0;
}







//-------------------- discrete differentiation utilies ------------------------------------



//will discretely differentiate position matrix and and return velocity matrix
//note that size changes from nq to nv
//expects a nq x t matrix q, a nv x t matrix v, and length t
void qpos2qvel(const mjModel* m, mjtNum* q, mjtNum* v, int len, mjtNum dt) {

    mjtNum q0[4], q0inv[4], q1[4], qdiff[4] = {1,0,0,0};
    mjtNum vel[3];
    
    int qID, vID, ii;
    
    for (int t=0; t<len; t++) {
        
        qID = 0; vID = 0;
        
        for (int jointID = 0; jointID < m->njnt; jointID++) {
            
            switch (m->jnt_type[jointID]) {
                    
                case mjJNT_SLIDE:
                case mjJNT_HINGE:
                    
                    if (t < len-1) {
                        
                        q0[0] = q[t*m->nq + qID];
                        q1[0] = q[(t+1)*m->nq + qID];
                        
                        
                        v[t*m->nv + vID] = (q1[0] - q0[0])/dt;
                    }
                    else {
                        v[t*m->nv + vID] =
                        2*v[m->nv*(t-1) + vID] - v[(t-2)*m->nv + vID];
                    }
                    
                    qID++; vID++;
                    
                    break;
                    
                case mjJNT_FREE:
                    
                    if (t < len-1) {
                        
                        //get xyz difference
                        v[t*m->nv + vID] =
                        (q[(t+1)*m->nq + qID] - q[t*m->nq + qID])/dt;
                        v[t*m->nv + vID + 1] =
                        (q[(t+1)*m->nq + qID + 1] - q[t*m->nq + qID + 1])/dt;
                        v[t*m->nv + vID + 2] =
                        (q[(t+1)*m->nq + qID + 2] - q[t*m->nq + qID + 2])/dt;
                        
                        //get this and future quat
                        mju_copy(q0,&q[t*m->nq + qID + 3],4);
                        mju_copy(q1,&q[(t+1)*m->nq + qID + 3],4);
                        
                        //calculate xyz angular velocities
                        mju_negQuat(q0inv, q0);
                        mju_mulQuat(qdiff, q0inv, q1);
                        mju_quat2Vel(vel, qdiff, dt);
                        
                        //then update velocity
                        mju_copy(&v[t*m->nv + vID + 3], vel, 3);
                    }
                    else { //extrapolate last velocity
                        
                        for (ii=vID; ii < vID+6; ii++) {
                            v[t*m->nv + ii] =
                            2*v[(t-1)*m->nv + ii] - v[(t-2)*m->nv + ii];
                        }
                    }
                    
                    qID += 7; vID += 6;
                    
                    break;
                    
                case mjJNT_BALL:
                    
                    if (t < len-1) {
                        
                        //get this and future quat
                        mju_copy(q0,&q[t*m->nq + qID],4);
                        mju_copy(q1,&q[(t+1)*m->nq + qID],4);
                        
                        //calculate xyz angular velocities
                        mju_negQuat(q0inv, q0);
                        mju_mulQuat(qdiff, q0inv, q1);
                        mju_quat2Vel(vel, qdiff, dt);
                        
                        //then update velocity
                        mju_copy(&v[t*m->nv + vID], vel, 3);
                    }
                    else { //extrapolate last velocity
                        
                        v[t*m->nv + vID] =
                        2*v[(t-1)*m->nv + vID] - v[(t-2)*m->nv + vID];
                        
                        v[t*m->nv + vID+1] =
                        2*v[(t-1)*m->nv + vID+1] - v[(t-2)*m->nv + vID+1];
                        
                        v[t*m->nv + vID+2] =
                        2*v[(t-1)*m->nv + vID+2] - v[(t-2)*m->nv + vID+2];
                    }
                    
                    
                    qID += 4; vID += 3;
                    
                    break;
            }
        }
    }
}


//differentiates velocity matrix to acceleration matrix
//size is preserved because ndof = ndof in both cases
void qvel2qacc(const mjModel* m, mjtNum* v, mjtNum* a, int len, mjtNum dt) {

    int t, dof;
    
    for(t = 0; t < len-1; t++) {
        
        for(dof=0; dof<m->nv; dof++) {
            
            a[t*m->nv + dof] =
            (v[(t+1)*m->nv + dof] - v[t*m->nv + dof]) / dt;
            
        }
    }
    
    //for last time point, assume acceleration remains constant
    //and just copy t-1 into t
    mju_copy(&a[t*m->nv], &a[(t-1)*m->nv], m->nv);
    
}








//-------------------------------- simulation utilies ------------------------------------


//runs mj_step for the duration of one frame (1/refreshrate)
void simulateFrame(mjModel* m, mjData* d)
{
    
    if (!paused) {
        // slow motion factor: 10x
        mjtNum factor = (slowmotion ? 10 : 1);
        
        // advance effective simulation time by 1/refreshrate
        mjtNum startsimtm = d->time;
        while ((d->time-startsimtm)*factor < 1.0/refreshrate) {
            
            //apply perturbations
            mju_zero(d->xfrc_applied, 6*m->nbody);
            if( pert.select>0 )
                mjv_applyPerturbForce(m, d, &pert);
            
            // run mj_step and count
            mj_step(m, d);
        }
    }
    else {
        // apply pose perturbations, run mj_forward
        mjv_applyPerturbPose(m, d, &pert, 1);       // move mocap and dynamic bodies
        mj_forward(m, d);
    }
}



//simple delay function
void waitSeconds(double delay)
{
    /* save start clock tick */
    const clock_t start = clock();
    
    clock_t current;
    do {
        /* get current clock tick */
        current = clock();
        
        /* break loop when the requested number of seconds have elapsed */
    } while((double)(current-start)/CLOCKS_PER_SEC < delay);
}



/* TODO
 
 //in glfw3.h, the following is defined as a key function
 //typedef void (* GLFWkeyfun)(GLFWwindow*,int,int,int,int);
 
 //make a function to take a function pointer and an event name
 //within the other keyboard/mouse callback funcs, override the
 //default case if the specified event occurs and instead point
 //to the specified custom function
 void setCustomCallback(GLFWwindow* window, const char* event, GLFWkeyfun customFunc)
 {
    //add "$event" to a list that the regular callback functions check
    //add the function pointer
 }
 */

#endif /* mujocoToolbox_h */
