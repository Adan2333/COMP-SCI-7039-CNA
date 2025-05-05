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
   - added SR implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the sequence space for SR must be at least 2*windowsize */
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

static struct pkt buffer[WINDOWSIZE];    /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;      /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                  /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;                 /* the next sequence number to be used by the sender */
static bool acked[WINDOWSIZE];           /* array to track which packets have been ACKed */
static int current_timer_seq;            /* sequence number of packet currently being timed */
static bool timer_running;               /* flag indicating if the timer is currently running */

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
    acked[windowlast] = false;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* Start timer if not already running */
    if (!timer_running) {
      current_timer_seq = sendpkt.seqnum;
      starttimer(A, RTT);
      timer_running = 1;
      if (TRACE > 0)
        printf("----A: Starting timer for packet %d\n", current_timer_seq);
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
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
  int i;
  int bufferIndex = -1;
  int idx;
  bool need_restart_timer = false;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* find which packet in our buffer this ACK corresponds to */
    for(i = 0; i < windowcount; i++) {
      idx = (windowfirst + i) % WINDOWSIZE;
      if(buffer[idx].seqnum == packet.acknum) {
        bufferIndex = idx;
        break;
      }
    }

    /* if ACK is for a packet in our window and not already ACKed */
    if (bufferIndex != -1 && !acked[bufferIndex]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
      new_ACKs++;

      /* mark this packet as acknowledged */
      acked[bufferIndex] = true;
      
      /* Check if the ACKed packet is the one currently being timed */
      if (timer_running && buffer[bufferIndex].seqnum == current_timer_seq) {
        if (TRACE > 0)
          printf("----A: Stopping timer for ACKed packet %d\n", current_timer_seq);
        stoptimer(A);
        timer_running = 0;
        need_restart_timer = true;
      }

      /* try to slide window if the first packet is ACKed */
      while (windowcount > 0 && acked[windowfirst]) {
        windowfirst = (windowfirst + 1) % WINDOWSIZE;
        windowcount--;
      }
      
      /* If we need to restart the timer or the timer isn't running, find next unacked packet */
      if ((need_restart_timer || !timer_running) && windowcount > 0) {
        for (i = 0; i < windowcount; i++) {
          idx = (windowfirst + i) % WINDOWSIZE;
          if (!acked[idx]) {
            if (TRACE > 0)
              printf("----A: Starting timer for next unacked packet %d\n", buffer[idx].seqnum);
            current_timer_seq = buffer[idx].seqnum;
            starttimer(A, RTT);
            timer_running = 1;
            break;
          }
        }
      }
    }
    else {
      if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else {
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  int next_unacked = -1;
  int next_unacked_idx = -1;

  if (TRACE > 0)
    printf("----A: time out for packet %d, resending it!\n", current_timer_seq);

  timer_running = 0;

  /* Find the buffer index for the timed-out packet */
  
  for(i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[idx] && buffer[idx].seqnum == current_timer_seq) {
      /* Resend this specific packet that timed out */
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
      
      /* Restart timer for this packet */
      starttimer(A, RTT);
      timer_running = 1;
      /* Current timer sequence stays the same */
      return;
    }
  }
  
  /* If we couldn't find the timed-out packet (it may have been ACKed and window moved),
     find the next unACKed packet and start timer for it */
  for(i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[idx]) {
      next_unacked = buffer[idx].seqnum;
      next_unacked_idx = idx;
      break;
    }
  }
  
  if (next_unacked_idx != -1) {
    if (TRACE > 0)
      printf ("---A: original packet already ACKed, resending next unacked packet %d\n", next_unacked);
    
    tolayer3(A, buffer[next_unacked_idx]);
    packets_resent++;
    
    current_timer_seq = next_unacked;
    starttimer(A, RTT);
    timer_running = 1;
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
  timer_running = 0;
  current_timer_seq = NOTINUSE;
  
  /* Initialize acked array */
 
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
  }
}

/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum;               /* the sequence number expected next by the receiver */
static int B_nextseqnum;                 /* the sequence number for the next packets sent by B */
static struct pkt rcv_buffer[WINDOWSIZE]; /* buffer for out-of-order but acceptable packets */
static bool received[WINDOWSIZE];         /* track which packets are received in the window */
static int rcv_base;                      /* base sequence number of receiver window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int bufferIndex;
  int seqnum;
  int relativeSeq;
  int next;

  /* if not corrupted and sequence number is within the receiver window */
  if (!IsCorrupted(packet)) {
    seqnum = packet.seqnum;
    /* Calculate if the seqnum is within our window (rcv_base to rcv_base+WINDOWSIZE-1) */
    if (seqnum >= rcv_base)
      relativeSeq = seqnum - rcv_base;
    else
      relativeSeq = SEQSPACE - rcv_base + seqnum;
    
    if (relativeSeq < WINDOWSIZE) {
      /* Packet is within our window */
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      packets_received++;
      
      /* Buffer the packet and mark as received */
      bufferIndex = relativeSeq;
      rcv_buffer[bufferIndex] = packet;
      received[bufferIndex] = true;
      
      /* Send ACK for this specific packet */
      sendpkt.acknum = seqnum;
      
      /* If this is the expected packet, deliver it and any consecutive packets */
      if (seqnum == expectedseqnum) {
        /* Deliver this packet */
        tolayer5(B, packet.payload);
        
        /* Update expectedseqnum and rcv_base */
        expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        rcv_base = expectedseqnum;
        
        /* Check if we have any consecutive packets already buffered */
        next = 1;
        while (received[next]) {
          /* Deliver this buffered packet */
          tolayer5(B, rcv_buffer[next].payload);
          
          /* Update state */
          received[next] = false;
          expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
          rcv_base = expectedseqnum;
          next = (next + 1) % WINDOWSIZE;
        }
        
        /* Shift the received array */
        for (i = 0; i < WINDOWSIZE - next; i++) {
          received[i] = received[i + next];
        }
        for (i = WINDOWSIZE - next; i < WINDOWSIZE; i++) {
          received[i] = false;
        }
      }
    } else {
      /* Packet is outside our window, but still send ACK if it's a duplicate */
      if (TRACE > 0) 
        printf("----B: packet out of window, resend ACK!\n");
      sendpkt.acknum = packet.seqnum;
    }
  } else {
    /* packet is corrupted */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    sendpkt.acknum = NOTINUSE;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out packet */
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
  
  /* Initialize received array */
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