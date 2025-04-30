#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"
#include <string.h>



/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - modified to implement Selective Repeat
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the sequence space for SR must be at least 2*windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

#define MAX_QUEUE_SIZE 50 

static struct msg message_queue[MAX_QUEUE_SIZE]; 
static int queue_front = 0; 
static int queue_rear = 0;  
static int queue_size = 0;  

/* SR protocol requires sequence space to be at least twice the window size */
#if SEQSPACE < 2*WINDOWSIZE
#error "SEQSPACE must be at least 2*WINDOWSIZE"
#endif

/* generic procedure to compute the checksum of a packet. */
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool acked[WINDOWSIZE];         /* array to track which packets have been ACKed */
static int timer_for_pkt;              /* keeps track of which packet timer is for */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;  /* Mark as not yet ACKed */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1) {
      starttimer(A, RTT);
      timer_for_pkt = windowfirst;  /* Timer is for the first packet in window */
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }

/* if blocked, window is full */
  else {
    if (TRACE > 0)
        printf("----A: New message arrives, send window is full. Message is queued.\n");

    /* Cache the message in the queue */
    if (queue_size < MAX_QUEUE_SIZE) {
        message_queue[queue_rear] = message;
        queue_rear = (queue_rear + 1) % MAX_QUEUE_SIZE;
        queue_size++;
    } else {
        if (TRACE > 0)
            printf("----A: Message queue is full, dropping message.\n");
    }

    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int pos;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if new ACK is within our window */
    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;

      /* check if ACK is within current window */
      bool inWindow = false;
      if (seqfirst <= seqlast) {
        inWindow = (packet.acknum >= seqfirst && packet.acknum <= seqlast);
      } else {
        inWindow = (packet.acknum >= seqfirst || packet.acknum <= seqlast);
      }

      if (inWindow) {
        /* Find position of this packet in the window buffer */
        pos = windowfirst;
        while (buffer[pos].seqnum != packet.acknum) {
          pos = (pos + 1) % WINDOWSIZE;
        }

        /* If this packet hasn't been ACKed yet */
        if (!acked[pos]) {
          if (TRACE > 0)
            printf("----A: ACK %d is new\n", packet.acknum);
          acked[pos] = true;
          new_ACKs++;

          /* If this is the first packet in the window and it's now ACKed,
             we can slide the window forward */
          if (pos == windowfirst) {
            /* Slide window past all consecutively ACKed packets */
            while (windowcount > 0 && acked[windowfirst]) {
              acked[windowfirst] = false; 
              windowfirst = (windowfirst + 1) % WINDOWSIZE;
              windowcount--;
            }

            /* Restart timer if needed */
            stoptimer(A);
            if (windowcount > 0) {
              starttimer(A, RTT);
              timer_for_pkt = windowfirst;
            }
          }
          while (queue_size > 0 && windowcount < WINDOWSIZE) {
            struct msg next_message = message_queue[queue_front];
            queue_front = (queue_front + 1) % MAX_QUEUE_SIZE;
            queue_size--;

            if (TRACE > 0)
                printf("----A: Sending queued message to layer3.\n");
            A_output(next_message); 
          }
        
        }else {
          if (TRACE > 0)
            printf("----A: duplicate ACK %d received, do nothing\n", packet.acknum);
        }
      }
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int next_pkt;
  int count;
  if (TRACE > 0)
    printf("----A: time out for packet %d, resend packet!\n", buffer[timer_for_pkt].seqnum);

  /* Resend only the packet whose timer expired */
  if (windowcount > 0) {
    tolayer3(A, buffer[timer_for_pkt]);
    packets_resent++;
    
    /* Restart timer */
    stoptimer(A);
    starttimer(A, RTT);
    
    /* For SR, we should move to the next unACKed packet */
    next_pkt = (timer_for_pkt + 1) % WINDOWSIZE;
    count = 0;
    /* Find the next unACKed packet to set timer for */
    while (count < windowcount && acked[next_pkt]) {
      next_pkt = (next_pkt + 1) % WINDOWSIZE;
      count++;
    }
    
    /* If all packets are ACKed, timer is for the first one */
    if (count == windowcount) {
      timer_for_pkt = windowfirst;
    } else {
      timer_for_pkt = next_pkt;
    }
  }
}       

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
                       new packets are placed in winlast + 1 
                       so initially this is set to -1
                     */
  windowcount = 0;
  timer_for_pkt = 0;
  
  /* Initialize the acked array */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
  }
}


/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;     /* the sequence number expected next by the receiver */
static int B_nextseqnum;       /* the sequence number for the next packets sent by B */
static int rcv_base;           /* base sequence number of receiver window */
static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[WINDOWSIZE]; /* tracks which packets in the window have been received */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i, pos,offset;

  /* if not corrupted and within receiver's window */
  if (!IsCorrupted(packet)) {
    /* Calculate where this packet belongs in the window */

    if (packet.seqnum >= rcv_base) {
      offset = packet.seqnum - rcv_base;
    } else {
      offset = (packet.seqnum - rcv_base + SEQSPACE) % SEQSPACE;
    }
    
    /* Check if the packet is within the window */
    if (offset < WINDOWSIZE) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received\n", packet.seqnum);
      
      /* Store the packet */
      pos = (rcv_base + offset) % WINDOWSIZE;
      recv_buffer[pos] = packet;
      received[pos] = true;
      
      /* If this is the expected packet, deliver it and any consecutive packets */
      if (packet.seqnum == expectedseqnum) {
        while (received[rcv_base % WINDOWSIZE]) {
          /* Deliver to application layer */
          tolayer5(B, recv_buffer[rcv_base % WINDOWSIZE].payload);
          packets_received++;
          
          /* Mark as no longer received */
          received[rcv_base % WINDOWSIZE] = false;
          
          /* Move window forward */
          rcv_base = (rcv_base + 1) % SEQSPACE;
          expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }
      }
      
      /* Send ACK for the received packet */
      sendpkt.acknum = packet.seqnum;
    } else {
      /* Packet is outside the window, but still ACK it if it's from the previous window */
      if (offset < 2 * WINDOWSIZE) {
        sendpkt.acknum = packet.seqnum;
      } else {
        /* Unexpected packet, don't ACK */
        if (TRACE > 0)
          printf("----B: packet %d is outside the window\n", packet.seqnum);
        return;
      }
    }
  } else {
    /* Packet is corrupted, don't send ACK */
    if (TRACE > 0)
      printf("----B: packet %d is corrupted\n", packet.seqnum);
    return;
  }

  /* create and send ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out packet */
  if (TRACE > 0)
    printf("----B: sending ACK %d\n", sendpkt.acknum);
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  rcv_base = 0;
  
  /* Initialize the received array */
  for (i = 0; i < WINDOWSIZE; i++) {
    received[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

