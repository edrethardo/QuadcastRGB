/* quadcastrgb - set RGB lights of HyperX Quadcast S and DuoCast
 * File devio.c
 *
 * <----- License notice ----->
 * Copyright (C) 2022, 2023, 2024 Ors1mer
 *
 * You may contact the author by email:
 * ors1mer [[at]] ors1mer dot xyz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License ONLY.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see
 * <https://www.gnu.org/licenses/gpl-2.0.en.html>. For any questions
 * concerning the license, you can write to <licensing@fsf.org>.
 * Also, you may visit the Free Software Foundation at
 * 51 Franklin Street, Fifth Floor Boston, MA 02110 USA. 
 */
#include <unistd.h> /* for usleep */
#include <fcntl.h> /* for daemonization */
#include <signal.h> /* for signal handling */
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/stat.h>

#include "locale_macros.h"

#include "devio.h"

/* Constants */

/*
#define DEV_PID_NA1     0x171f
#define DEV_PID_EU1     0x0f8b
#define DEV_PID_EU2     0x028c
#define DEV_PID_EU3     0x048c
#define DEV_PID_EU4     0x068c
#define DEV_PID_DUOCAST 0x098c
#define DEV_PID_QC2S    0x02b5
*/

#define DEV_EPOUT 0x00 /* control endpoint OUT */
#define DEV_EPIN 0x80 /* control endpoint IN */
/* Packet info */
#define MAX_PCT_CNT 90
#define PACKET_SIZE 64 /* bytes */

#define HEADER_CODE 0x04
#define DISPLAY_CODE 0xf2
#define PACKET_CNT 0x01

#define INTR_EP_IN 0x82
#define INTR_LENGTH 8

#define TIMEOUT 1000 /* one second per packet */
#define BMREQUEST_TYPE_OUT 0x21
#define BREQUEST_OUT 0x09
#define BMREQUEST_TYPE_IN 0xa1
#define BREQUEST_IN 0x01
#define WVALUE 0x0300
#define WINDEX 0x0000
/* Messages */
#define DEVLIST_ERR_MSG _("Couldn't get the list of USB devices.\n")
#define NODEV_ERR_MSG _("HyperX Quadcast S isn't connected.\n")
#define OPEN_ERR_MSG _("Couldn't open the microphone.\n")
#define BUSY_ERR_MSG _("Another program is using the microphone already. " \
                       "Stopping.\n")
#define TRANSFER_ERR_MSG _("Couldn't transfer a packet! " \
                           "The device might be busy.\n")
#define FOOTER_ERR_MSG _("Footer packet error: %s\n")
#define HEADER_ERR_MSG _("Header packet error: %s\n")
#define SIZEPCK_ERR_MSG _("Size packet error: %s\n")
#define DATAPCK_ERR_MSG _("Data packet error: %s\n")
#define PID_MSG _("Started with pid %d\n")
#define SOCKET_PATH "/tmp/quadcastrgb.sock"

/* Error codes */
enum {
    libusberr = 2,
    nodeverr,
    devopenerr,
    transfererr
};

/* For open_micro */
#define FREE_AND_EXIT() \
    libusb_free_device_list(devs, 1); \
    free(data_arr); \
    libusb_exit(NULL); \
    exit(libusberr)

#define HANDLE_ERR(CONDITION, MSG) \
    if(CONDITION) { \
        fprintf(stderr, MSG); \
        FREE_AND_EXIT(); \
    }
/* For send_packets */
#define HANDLE_TRANSFER_ERR(ERRCODE) \
    if(ERRCODE) { \
        fprintf(stderr, TRANSFER_ERR_MSG); \
        libusb_close(handle); \
        libusb_exit(NULL); \
        free(data_arr); \
        exit(transfererr); \
    }

/* Vendor IDs */
#define DEV_VID_KINGSTON      0x0951
#define DEV_VID_HP            0x03f0
/* Product IDs */
const unsigned short product_ids_kingston[] = {
    0x171f
};
const unsigned short product_ids_hp[] = {
    0x0f8b,
    0x028c,
    0x048c,
    0x068c,
    0x098c  /* Duocast */
};

/* Microphone opening */
static int claim_dev_interface(libusb_device_handle *handle);
static libusb_device *dev_search(libusb_device **devs, ssize_t cnt);
static int is_compatible_mic(libusb_device *dev);
/* Packet transfer */
static short send_display_command(byte_t *packet,
                                  libusb_device_handle *handle);
static void display_data_arr(libusb_device_handle *handle,
                             datpack **data_ptr, int *cnt_ptr, int *is_dyn_ptr,
                             int server_sock);
#ifndef DEBUG
static void daemonize(int verbose);
#endif
#ifdef DEBUG
static void print_packet(byte_t *pck, char *str);
#endif

/* Signal handling */
volatile static sig_atomic_t nonstop = 0; /* BE CAREFUL: GLOBAL VARIABLE */
static void nonstop_reset_handler(int s)
{
    /* No need in saving errno or setting the handler again
     * because the program just frees memory and exits */
    nonstop = 0;
}

/* Functions */
int send_packet_to_socket(datpack *data_arr, int pck_cnt)
{
    int sock;
    struct sockaddr_un addr;
    int ret = 0;

    sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) return 0;

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        close(sock);
        return 0; /* Daemon not running or not listening */
    }

    /* Send packet count first */
    if (write(sock, &pck_cnt, sizeof(int)) != sizeof(int)) ret = 1;
    /* Send data */
    else if (write(sock, data_arr, sizeof(datpack) * pck_cnt) != sizeof(datpack) * pck_cnt) ret = 1;

    close(sock);
    return ret == 0;
}

libusb_device_handle *open_micro(datpack *data_arr)
{


    libusb_device **devs;

    libusb_device *micro_dev = NULL;
    libusb_device_handle *handle;
    ssize_t dev_count;
    short errcode;

    /* Removed data_packet_cnt dependency here since open_micro
       signature doesn't have it, but we need it for socket send.
       However, the caller has it. We'll modify logic:
       If USB is busy, we check if we can talk to the daemon. */
    
    errcode = libusb_init(NULL);
    if(errcode) {
        perror("libusb_init");
        free(data_arr); exit(libusberr);
    }
    dev_count = libusb_get_device_list(NULL, &devs);
    HANDLE_ERR(dev_count < 0, DEVLIST_ERR_MSG);
    micro_dev = dev_search(devs, dev_count);
    HANDLE_ERR(!micro_dev, NODEV_ERR_MSG);
    errcode = libusb_open(micro_dev, &handle);
    if(errcode) {
        fprintf(stderr, "%s\n%s", libusb_strerror(errcode), OPEN_ERR_MSG);
        FREE_AND_EXIT();
    }
    errcode = claim_dev_interface(handle);
    if(errcode == 2) {
        /* Device busy. Try sending data to daemon. */
        /* But we need data_arr here. */
        /* We need to pass data_arr and count to open_micro, OR handle it outside. 
           But checking for BUSY happens deep inside. 
           Let's rely on global 'datpack *data_arr' passed to this function. 
           But we don't have the PACKET COUNT here. 'datpack *data_arr' is just a pointer. */
           
        /* Re-design: open_micro shouldn't take responsibility for fallback to socket if it doesn't have all info. 
           However, we can just return NULL or error, and let main() handle the fallback?
           Main has all info. */
        
        libusb_close(handle); 
        libusb_free_device_list(devs, 1);
        return NULL; /* Return NULL to indicate failure to open (potentially busy) */
    } else if(errcode) {
        libusb_close(handle); FREE_AND_EXIT();
    }
    libusb_free_device_list(devs, 1);
    return handle;
}

/* Forward declaration */
/* static int send_packet_to_socket(datpack *data_arr, int pck_cnt); RE MOVED */

static int claim_dev_interface(libusb_device_handle *handle)
{
    int errcode0, errcode1;
    libusb_set_auto_detach_kernel_driver(handle, 1); /* might be unsupported */
    if (libusb_kernel_driver_active(handle, 0) == 1) {
        libusb_detach_kernel_driver(handle, 0);
    }
    if (libusb_kernel_driver_active(handle, 1) == 1) {
        libusb_detach_kernel_driver(handle, 1);
    }
    errcode0 = libusb_claim_interface(handle, 0);
    errcode1 = libusb_claim_interface(handle, 1);
    if(errcode0 == LIBUSB_ERROR_BUSY || errcode1 == LIBUSB_ERROR_BUSY) {
        /* Instead of just failing, we return a SPECIFIC error code that allows checking for daemon */
        return 2; /* 2 = Busy */
    } else if(errcode0 == LIBUSB_ERROR_NO_DEVICE ||
                                          errcode1 == LIBUSB_ERROR_NO_DEVICE) {
        fprintf(stderr, OPEN_ERR_MSG);
        return 1;
    }
    return 0;
}

static libusb_device *dev_search(libusb_device **devs, ssize_t cnt)
{
    libusb_device **dev;
    for(dev = devs; dev < devs+cnt; dev++) {
        if(is_compatible_mic(*dev))
            return *dev;
    }
    return NULL;
}

static int is_compatible_mic(libusb_device *dev)
{
    int i, arr_size;
    const unsigned short *product_id_arr;
    struct libusb_device_descriptor descr;
    libusb_get_device_descriptor(dev, &descr);

    if (descr.idVendor == DEV_VID_KINGSTON) {
        product_id_arr = product_ids_kingston;
        arr_size = sizeof(product_ids_kingston)/sizeof(*product_id_arr);
    } else if (descr.idVendor == DEV_VID_HP) {
        product_id_arr = product_ids_hp;
        arr_size = sizeof(product_ids_hp)/sizeof(*product_id_arr);
    } else {
        return 0;
    }

    #ifdef DEBUG
    printf("Valid vendor found: %04x\nTrying product ids:\n", descr.idVendor);
    #endif
    for (i = 0; i < arr_size; i++) {
        #ifdef DEBUG
        printf("\t%04x\n", product_id_arr[i]);
        #endif
        if (descr.idProduct == product_id_arr[i])
            return 1;
    }
    return 0;
}

void send_packets(libusb_device_handle *handle, const datpack *data_arr,
                  int pck_cnt, int verbose)
{
    int server_sock = -1;
    struct sockaddr_un addr;
    
    /* State management */
    datpack *current_data = (datpack *)data_arr;
    int current_pck_cnt = pck_cnt;
    int is_dynamic = 0; /* 0 = owned by main/caller, 1 = owned by heap */

    /* Setup socket server before daemonizing */
    unlink(SOCKET_PATH);
    server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_sock >= 0) {
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
        if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(server_sock);
            server_sock = -1;
        } else {
            listen(server_sock, 5);
            /* Make socket permissions accessible */
            chmod(SOCKET_PATH, 0777);
        }
    }

    #ifdef DEBUG
    puts("Entering display mode...");
    #endif
    #ifndef DEBUG
    daemonize(verbose);
    #endif
    
    signal(SIGINT, nonstop_reset_handler);
    signal(SIGTERM, nonstop_reset_handler);
    /* The loop works until a signal handler resets the variable */
    nonstop = 1; 

    while(nonstop) {
        /* Pass addresses of state variables */
        display_data_arr(handle, &current_data, &current_pck_cnt, &is_dynamic, server_sock);
    }
    
    if (is_dynamic) {
        free(current_data);
    }
    
    if (server_sock >= 0) {
        close(server_sock);
        unlink(SOCKET_PATH);
    }
}

#ifndef DEBUG
static void daemonize(int verbose)
{
    int pid;

    if (chdir("/") < 0) { }
    pid = fork();
    if(pid > 0)
        exit(0);
    setsid();
    pid = fork();
    if(pid > 0)
        exit(0);

    if(verbose)
        printf(PID_MSG, getpid()); /* notify the user */
    fflush(stdout); /* force clear of the buffer */
    close(0);
    close(1);
    close(2);
    open("/dev/null", O_RDONLY);
    open("/dev/null", O_WRONLY);
    open("/dev/null", O_WRONLY);
}
#endif

static void display_data_arr(libusb_device_handle *handle,
                             datpack **data_ptr, int *cnt_ptr, int *is_dyn_ptr,
                             int server_sock)
{
    short sent;
    short command_cnt = count_color_commands(*data_ptr, *cnt_ptr, 0);
    const byte_t *start_of_data = (const byte_t *)*data_ptr;
    const byte_t *colcommand = start_of_data;
    const byte_t *end = start_of_data + 2*BYTE_STEP*command_cnt;
    
    byte_t *packet;
    byte_t header_packet[PACKET_SIZE] = {
        HEADER_CODE, DISPLAY_CODE, 0, 0, 0, 0, 0, 0, PACKET_CNT, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    
    /* Socket variables */
    struct timeval tv;
    fd_set readfds;
    int activity, new_sock;
    datpack *new_data_arr = NULL;
    int new_pck_cnt;
    ssize_t bytes_read;
    
    packet = calloc(PACKET_SIZE, 1);
    
    while(colcommand < end && nonstop) {
        /* Check for new connections */
        if (server_sock >= 0) {
            FD_ZERO(&readfds);
            FD_SET(server_sock, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 0; /* Non-blocking */
            
            activity = select(server_sock + 1, &readfds, NULL, NULL, &tv);
            if (activity > 0 && FD_ISSET(server_sock, &readfds)) {
                new_sock = accept(server_sock, NULL, NULL);
                if (new_sock >= 0) {
                    /* Read packet count */
                    if (read(new_sock, &new_pck_cnt, sizeof(int)) == sizeof(int)) {
                        /* Allocate and read new data */
                        new_data_arr = malloc(sizeof(datpack) * new_pck_cnt);
                        if (new_data_arr) {
                            int total_read = 0;
                            int target_size = sizeof(datpack) * new_pck_cnt;
                            while (total_read < target_size) {
                                bytes_read = read(new_sock, ((char*)new_data_arr) + total_read, target_size - total_read);
                                if (bytes_read <= 0) break;
                                total_read += bytes_read;
                            }
                            
                            if (total_read == target_size) {
                                /* Success! Update state and return immediately to restart loop */
                                if (*is_dyn_ptr) {
                                    free(*data_ptr);
                                }
                                *data_ptr = new_data_arr;
                                *cnt_ptr = new_pck_cnt;
                                *is_dyn_ptr = 1;
                                
                                close(new_sock);
                                free(packet);
                                return; /* Breaking function, will be called again with NEW data */
                            } else {
                                free(new_data_arr);
                            }
                        }
                    }
                    close(new_sock);
                }
            }
        }

        sent = send_display_command(header_packet, handle);
        if(sent != PACKET_SIZE) {
            nonstop = 0; break;
        }
        memcpy(packet, colcommand, 2*BYTE_STEP);
        sent = libusb_control_transfer(handle, BMREQUEST_TYPE_OUT,
                   BREQUEST_OUT, WVALUE, WINDEX, packet, PACKET_SIZE, TIMEOUT);
        if(sent != PACKET_SIZE) {
            nonstop = 0; break;
        }
        #ifdef DEBUG
        print_packet(packet, "Data:");
        #endif
        colcommand += 2*BYTE_STEP;
        usleep(1000*55);
    }
    free(packet);
}

static short send_display_command(byte_t *packet, libusb_device_handle *handle)
{
    short sent;
    sent = libusb_control_transfer(handle, BMREQUEST_TYPE_OUT, BREQUEST_OUT,
                                 WVALUE, WINDEX, packet, PACKET_SIZE,
                                 TIMEOUT);
    #ifdef DEBUG
    print_packet(packet, "Header display:");
    if(sent != PACKET_SIZE)
        fprintf(stderr, HEADER_ERR_MSG, libusb_strerror(sent));
    #endif
    return sent;
}

#ifdef DEBUG
static void print_packet(byte_t *pck, char *str)
{
    byte_t *p;
    puts(str);
    for(p = pck; p < pck+PACKET_SIZE; p++) {
        printf("%02X ", (int)(*p));
        if((p-pck+1) % 16 == 0)
            puts("");
    }
    puts("");
}
#endif
