#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from GBN implementation.
   
   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the sequence space for SR must be at least 2 * windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
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

static struct pkt buffer[WINDOWSIZE];      /* array for storing packets waiting for ACK */
static int A_windowbase;                   /* base sequence number of the window */
static int A_nextseqnum;                   /* the next sequence number to be used by the sender */
static int packet_timer[WINDOWSIZE];       /* timer status for each packet in window */
static int windowcount;                    /* the number of packets currently awaiting an ACK */
static int timer_running;                  /* flag to track if timer is running */
static int oldest_unacked;                 /* index of the oldest unacked packet */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int window_index;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    window_index = A_nextseqnum % WINDOWSIZE;
    buffer[window_index] = sendpkt;
    packet_timer[window_index] = 0;  /* not acked yet */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if not already running */
    if (!timer_running) {
      starttimer(A, RTT);
      timer_running = 1;
      oldest_unacked = window_index;
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int window_index;
  int i;
  int all_acked;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if the ACK is within our current window */
    if (((A_windowbase <= A_nextseqnum) && 
         (packet.acknum >= A_windowbase && packet.acknum < A_nextseqnum)) ||
        ((A_windowbase > A_nextseqnum) && 
         (packet.acknum >= A_windowbase || packet.acknum < A_nextseqnum))) {
      
      /* packet is a new ACK within our window */
      window_index = packet.acknum % WINDOWSIZE;
      
      /* check if this packet hasn't been acked yet */
      if (packet_timer[window_index] == 0) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        /* mark packet as acknowledged */
        packet_timer[window_index] = 1;
        
        /* check if we can advance window */
        while (packet_timer[A_windowbase % WINDOWSIZE] == 1 && windowcount > 0) {
          /* advance window */
          packet_timer[A_windowbase % WINDOWSIZE] = -1; /* mark as outside window */
          A_windowbase = (A_windowbase + 1) % SEQSPACE;
          windowcount--;
        }
        
        /* check if timer needs to be restarted */
        if (windowcount > 0) {
          /* find the oldest unacked packet */
          all_acked = 1;
          for (i = 0; i < WINDOWSIZE; i++) {
            if (packet_timer[i] == 0) {
              all_acked = 0;
              oldest_unacked = i;
              break;
            }
          }
          
          if (!all_acked) {
            /* restart timer for the oldest unacked packet */
            stoptimer(A);
            starttimer(A, RTT);
          } else {
            /* all packets in window are acked, stop timer */
            stoptimer(A);
            timer_running = 0;
          }
        } else {
          /* window is empty, stop timer */
          stoptimer(A);
          timer_running = 0;
        }
      } 
      else {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      }
    }
    else {
      if (TRACE > 0)
        printf("----A: ACK outside current window, do nothing!\n");
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
  int i;
  int next_unacked = -1;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* find the oldest unacked packet and resend it */
  if (packet_timer[oldest_unacked] == 0) {
    if (TRACE > 0)
      printf("---A: resending packet %d\n", buffer[oldest_unacked].seqnum);
    
    tolayer3(A, buffer[oldest_unacked]);
    packets_resent++;
  }
  
  /* find next unacked packet for future timeouts */
  for (i = (oldest_unacked + 1) % WINDOWSIZE; 
       i != oldest_unacked; 
       i = (i + 1) % WINDOWSIZE) {
    if (packet_timer[i] == 0) {
      next_unacked = i;
      break;
    }
  }
  
  if (next_unacked != -1) {
    /* found another unacked packet, update oldest_unacked and restart timer */
    oldest_unacked = next_unacked;
    starttimer(A, RTT);
  } else if (packet_timer[oldest_unacked] == 0) {
    /* only the current oldest is still unacked, restart timer for it */
    starttimer(A, RTT);
  } else {
    /* no unacked packets, stop timer */
    timer_running = 0;
  }
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  A_windowbase = 0;
  windowcount = 0;
  timer_running = 0;
  oldest_unacked = 0;
  
  /* initialize all packet timers to -1 (not in use) */
  for (i = 0; i < WINDOWSIZE; i++) {
    packet_timer[i] = -1;
  }
}



/********* Receiver (B) variables and procedures ************/

static int B_windowbase;                    /* base sequence number of the receive window */
static struct pkt recv_buffer[WINDOWSIZE];  /* buffer for out-of-order packets */
static int packet_received[WINDOWSIZE];     /* track if packet at this position is received */
static int B_nextseqnum;                    /* sequence number for next packet from B */


/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int window_index;
  
  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    /* check if the packet is within our current window */
    if (((B_windowbase <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.seqnum >= B_windowbase && packet.seqnum <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE)) ||
        ((B_windowbase > (B_windowbase + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.seqnum >= B_windowbase || packet.seqnum <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE))) {
      
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      
      /* send ACK for the received packet */
      sendpkt.acknum = packet.seqnum;
      
      /* store the packet in the receive buffer */
      window_index = (packet.seqnum - B_windowbase + SEQSPACE) % SEQSPACE % WINDOWSIZE;
      if (!packet_received[window_index]) {
        /* only count as received if we haven't received it before */
        packets_received++;
        recv_buffer[window_index] = packet;
        packet_received[window_index] = 1;
      }
      
      /* check if we can deliver packets in-order */
      while (packet_received[0]) {
        /* deliver to receiving application */
        tolayer5(B, recv_buffer[0].payload);
        
        /* slide window */
        for (i = 0; i < WINDOWSIZE - 1; i++) {
          packet_received[i] = packet_received[i + 1];
          recv_buffer[i] = recv_buffer[i + 1];
        }
        packet_received[WINDOWSIZE - 1] = 0;
        
        /* advance window base */
        B_windowbase = (B_windowbase + 1) % SEQSPACE;
      }
    }
    else {
      /* packet is outside the window */
      /* if it's a packet we've already ACKed (before our window base) */
      if (((B_windowbase - WINDOWSIZE + SEQSPACE) % SEQSPACE <= B_windowbase - 1) && 
          (packet.seqnum >= (B_windowbase - WINDOWSIZE + SEQSPACE) % SEQSPACE && 
           packet.seqnum <= B_windowbase - 1)) {
        if (TRACE > 0)
          printf("----B: packet %d was already ACKed, resend ACK!\n", packet.seqnum);
        
        /* resend ACK for already received packet */
        sendpkt.acknum = packet.seqnum;
      }
      else {
        /* packet is too far ahead or too far behind our window */
        if (TRACE > 0)
          printf("----B: packet outside receive window, do nothing!\n");
          
        return;  /* don't send an ACK for packets too far out of window */
      }
    }
  }
  else {
    /* packet is corrupted */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
      
    return;  /* don't send an ACK for corrupted packets */
  }

  /* create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  B_windowbase = 0;
  B_nextseqnum = 1;
  
  /* initialize receive buffer */
  for (i = 0; i < WINDOWSIZE; i++) {
    packet_received[i] = 0;
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