//
//  ethraw.h
//
//
//  Created by Enrico Eberhard on 26/09/2017.
//
//  This file constitutes a hopefully cross-platform
//	utility for sending and receiving raw ethernet packets.

//	On Linux, this is done by opening a raw socket and
//	manually configuring the packet.
//
//	With help from:
//	http://hacked10bits.blogspot.co.uk/2011/12/sending-raw-ethernet-frames-in-6-easy.html
//
//  On OSX, raw sockets require IP headers. Instead, we
//	operate directly on a BPF device.
//
//	With help from:
//	http://bastian.rieck.ru/howtos/bpf/


//  main utilities should be send_packet and read_packet,
//	and should automatically compile to use distinct methods
//	based on platform


#ifndef ethraw_h
#define ethraw_h


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


//PLATFORM DEPENDENT

#ifdef __linux__

#include <linux/if_packet.h>
#include <netinet/ether.h>
#include <sys/mman.h>
extern struct sockaddr_ll this_sockaddr;

#elif __APPLE__

#include <sys/types.h>
#include <sys/uio.h>
#include <fcntl.h>
#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if_dl.h>
#include <ifaddrs.h>

#endif


struct ethframe {
    struct ether_header header;
    unsigned char data[ETHER_MAX_LEN - ETHER_HDR_LEN];
    ssize_t len;
    ssize_t data_len;
};


// Some convenience constants
extern const size_t ETHER_PAYLOAD_LEN;

//global vars to be defined in source file
// (making them extern lets compiler set default values)
extern int bpf_buf_len;

extern char interface[IFNAMSIZ];

//mac addresses
extern char dest_mac_str[];
extern unsigned char dest_mac[ETHER_ADDR_LEN];
extern unsigned char src_mac[ETHER_ADDR_LEN];
//interface protocol
extern unsigned short protocol;



/* Public Function Prototypes */


// set up the default destination address, device name and protocol
void set_eth_defaults(const char* dest_, const char* iface_,
                      unsigned short proto_);

// initialization routine that runs open_interface and associate_interface
// according to defaults. Returns a device ID to be used as inputs for
// send_frame and read_frames
int initialize_eth_device(void);

// write data to an ethernet frame and send through interface
int send_frame(int device, unsigned char* data, size_t data_len);

// read frames from an interface and pass data to callback function
void read_frames(int device, void (*callback)(unsigned char*));


#endif /* ethraw_h */
