#ifndef RASPISERIAL_H
#define RASPISERIAL_H

#include <pthread.h>
#include "stdint.h"
#include "time.h"
#include <vector>
#include <iostream>
#include <unistd.h> //usleep
#include <termios.h>


#include "wiringPi.h"
#include "wiringSerial.h"


#define STATE_CONNECTION_CLOSED  0x00
#define STATE_WAITING_FOR_JOBS   0x01
#define STATE_IDLE               0x02

#define RECEIVE_BUFFER_SIZE 1024
#define RECEIVE_BUFFER_SIZE_MASK (RECEIVE_BUFFER_SIZE-1)


class RaspiSerial
{
private:
    typedef struct job_type {
    char*            data_ptr;
    uint32_t        data_length;
    }   job_type;

    typedef struct rec_buf_type {
    uint16_t         write_ptr;
    uint16_t         read_ptr;
    uint8_t          buf [RECEIVE_BUFFER_SIZE];
    }   rec_buf_type;


    int                      m_serial_fdescriptor;
    pthread_t                thread_serial;
    uint8_t                  thread_state;
    bool*                    start_flag;
    pthread_mutex_t          job_list_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t          thread_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    rec_buf_type             m_receive_buffer;
    int32_t                 m_baudrate;
    uint16_t                 m_bytes_available;
    std::vector<job_type>    job_queue;

public:
    RaspiSerial(int32_t baudrate)
    {
        m_baudrate = baudrate;

        if (pthread_create(&this->thread_serial, NULL, (THREADFUNCPTR) &RaspiSerial::serialThread, this))
            std::cout<<"now we should quit"; //quit("\n--> pthread_create failed.", 1);
    }

//    void replyDelayExpired ( void )  __attribute__((weak))
//    {

//    }

    void add_job (char* t_data_ptr, uint32_t t_data_length)
    {
        this->job_queue.insert( job_queue.begin(),  { t_data_ptr, t_data_length });
    }

    void close_connection ( void )
    {
        pthread_mutex_lock(&thread_state_mutex);
        std::cout<<"closing Serial";
        thread_state = STATE_CONNECTION_CLOSED;
        pthread_mutex_unlock(&thread_state_mutex);

    }

private:
    typedef void * (*THREADFUNCPTR)(void *);

    void* serialThread(void* arg)
    {
        /* make this thread cancellable using pthread_cancel() */
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

        int16_t bytes_sent;
        static job_type t_job;
        static int16_t bytes_available;

        if ((m_serial_fdescriptor = serialOpen ("/dev/ttyAMA0", m_baudrate)) < 0)
        {
          std::cout<<"Cant Open Serial Port..";
            //fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        }

        if (wiringPiSetup() == -1)
        {
          serialClose(m_serial_fdescriptor);
          std::cout<<"Cant Setup WiringPi..";
            //fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        }

        serialPrintf (m_serial_fdescriptor, "Serial is Alive!\n") ;
        std::cout<<"Thread Started"<<std::endl;

        thread_state = STATE_WAITING_FOR_JOBS;

        /* start serial thread */
        while(1)
        {
            pthread_mutex_lock(&thread_state_mutex);
            switch (thread_state)
            {
            case STATE_WAITING_FOR_JOBS:
                // SEND IF THERE IS STUFF TO SEND)
                if (job_queue.size())
                {
                    pthread_mutex_lock(&job_list_mutex);

                    t_job = job_queue.back();
                    job_queue.pop_back();

                    if ((bytes_sent = write (m_serial_fdescriptor, t_job.data_ptr, t_job.data_length) ) < 0){
                        close_connection();
                        std::cout<<"Send Failed!";
                    }
                    pthread_mutex_unlock(&job_list_mutex);

                    /* if something went wrong, restart the connection */
                    if ((uint32_t)bytes_sent != t_job.data_length)
                    {
                        close_connection();
                    }
                }

                // RECEIVE IF THERE IS STUFF TO RECEIVE)
                bytes_available = serialDataAvail (m_serial_fdescriptor);
                if ( bytes_available > 0 )
                {
                    for (int i = 0; i < bytes_available; i++)
                    {
                        m_receive_buffer.buf[m_receive_buffer.write_ptr++ & RECEIVE_BUFFER_SIZE_MASK] =
                                serialGetchar(m_serial_fdescriptor);
                    }
                    bytes_available = 0;
                }
                break;
            case STATE_IDLE:
                serialFlush(m_serial_fdescriptor);
                break;
            }
            pthread_mutex_unlock(&thread_state_mutex);
            /* have we terminated yet? */
            pthread_testcancel();
            /* no, take a rest for a while */
            usleep(100);   //1000 Micro Sec
        }
    }
};

#endif // RASPISERIAL_H
