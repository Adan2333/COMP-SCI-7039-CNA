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
   - added SACK implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the sequence space for SR must be at least 2*windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */
#define MAX_SACK 6      /* Maximum number of packets that can be SACKed in one ACK */

/* SACK structure to hold multiple ACKs */
struct sack_info {
    int count;                  /* Number of packets being SACKed */
    int seqnums[MAX_SACK];      /* Array of sequence numbers being SACKed */
};

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

/* Functions to encode and decode SACK information in the packet payload */
void EncodeSACK(struct pkt *packet, struct sack_info sack)
{
    char sackData[20]; /* Buffer to store SACK encoding */
    int i;
    
    /* Format: First byte is count, followed by sequence numbers */
    sackData[0] = '0' + sack.count; /* Convert count to character */
    
    /* Store each sequence number - we'll use 2 bytes per number for simplicity */
    for (i = 0; i < sack.count && i < MAX_SACK; i++) {
        sackData[1 + i*2] = '0' + (sack.seqnums[i] / 10); /* Tens digit */
        sackData[2 + i*2] = '0' + (sack.seqnums[i] % 10); /* Ones digit */
    }
    
    /* Fill the rest with '0' */
    for (i = 1 + sack.count*2; i < 20; i++) {
        sackData[i] = '0';
    }
    
    /* Copy to packet payload */
    memcpy(packet->payload, sackData, 20);
}

struct sack_info DecodeSACK(struct pkt packet)
{
    struct sack_info sack;
    int i;
    
    /* Get count from first byte */
    sack.count = packet.payload[0] - '0';
    
    /* Bounds check */
    if (sack.count > MAX_SACK || sack.count < 0) {
        sack.count = 0; /* Invalid SACK */
        return sack;
    }
    
    /* Extract each sequence number */
    for (i = 0; i < sack.count; i++) {
        int tens = packet.payload[1 + i*2] - '0';
        int ones = packet.payload[2 + i*2] - '0';
        
        /* Validate digits */
        if (tens < 0 || tens > 9 || ones < 0 || ones > 9) {
            sack.count = 0; /* Invalid SACK format */
            return sack;
        }
        
        sack.seqnums[i] = tens * 10 + ones;
    }
    
    return sack;
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];    /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;      /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                  /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;                 /* the next sequence number to be used by the sender */
static bool acked[WINDOWSIZE];           /* array to track which packets have been ACKed */
static int timers[WINDOWSIZE];           /* array to track which packet's timer is running */
static int timer_running;                /* Track if any timer is running */

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

    if (!timer_running) {
      timers[windowlast] = A_nextseqnum;
      starttimer(A, RTT);
      timer_running = 1; 
    } else {
      timers[windowlast] = A_nextseqnum;
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
  int i, j;
  int bufferIndex = -1;
  int idx;
  int next_timer_idx = -1;
  struct sack_info sack;
  bool window_updated = false;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* Decode SACK information */
    sack = DecodeSACK(packet);
    
    /* Process primary ACK (packet.acknum) */
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
      if (timers[bufferIndex] != NOTINUSE) {
        stoptimer(A);
        timer_running = 0;
        timers[bufferIndex] = NOTINUSE;
      }
      
      window_updated = true;
    }
    else {
      if (TRACE > 0)
        printf ("----A: duplicate ACK received, checking SACK info!\n");
    }
    
    /* Process SACK information */
    for (i = 0; i < sack.count; i++) {
      /* find which packet in our buffer this SACK corresponds to */
      bufferIndex = -1;
      for(j = 0; j < windowcount; j++) {
        idx = (windowfirst + j) % WINDOWSIZE;
        if(buffer[idx].seqnum == sack.seqnums[i] && !acked[idx]) {
          bufferIndex = idx;
          break;
        }
      }
      
      /* if SACK is for a packet in our window and not already ACKed */
      if (bufferIndex != -1) {
        if (TRACE > 0)
          printf("----A: SACK for packet %d received\n", sack.seqnums[i]);
        
        /* mark this packet as acknowledged */
        acked[bufferIndex] = true;
        if (timers[bufferIndex] != NOTINUSE) {
          stoptimer(A);
          timer_running = 0;
          timers[bufferIndex] = NOTINUSE;
        }
        
        window_updated = true;
        new_ACKs++; /* Count SACK as new ACK for statistics */
      }
    }
    
    /* If window was updated by any ACK or SACK, try to slide window */
    if (window_updated) {
      /* try to slide window if the first packet is ACKed */
      while (windowcount > 0 && acked[windowfirst]) {
        windowfirst = (windowfirst + 1) % WINDOWSIZE;
        windowcount--;
      }
      
      /* Start timer for the first unacked packet if needed */
      if (!timer_running && windowcount > 0) {
        for (i = 0; i < windowcount; i++) {
          idx = (windowfirst + i) % WINDOWSIZE;
          if (!acked[idx]) {
            next_timer_idx = idx;
            break;
          }
        }
        
        if (next_timer_idx != -1) {
          starttimer(A, RTT);
          timer_running = 1;
        }
      }
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
  int first_unacked = -1;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  timer_running = 0;

  for(i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    
    if (!acked[idx]) {
      first_unacked = idx;
      break;
    }
  }
  
  /* not sure, resend */
  if (first_unacked != -1) {
    if (TRACE > 0)
      printf ("---A: resending packet %d\n", buffer[first_unacked].seqnum);
    
    tolayer3(A, buffer[first_unacked]);
    packets_resent++;
    
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
  timer_running = 0; /* Initialize timer_running to 0 (no timer running) */
  
  /* Initialize acked and timers arrays */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = NOTINUSE;
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
  struct sack_info sack;
  static int total_packets_received = 0; 

  /* Initialize SACK info */
  sack.count = 0;

  /* if not corrupted and sequence number is within the receiver window */
  if (!IsCorrupted(packet)) {
    total_packets_received++;
    seqnum = packet.seqnum;
    
    /* Calculate if the seqnum is within our window (rcv_base to rcv_base+WINDOWSIZE-1) */
    /* Handle sequence number wraparound correctly */
    if ((rcv_base <= seqnum && seqnum < rcv_base + WINDOWSIZE) || 
        (rcv_base + WINDOWSIZE >= SEQSPACE && seqnum < (rcv_base + WINDOWSIZE) % SEQSPACE)) {
      if (seqnum >= rcv_base)
        relativeSeq = seqnum - rcv_base;
    else
        relativeSeq = SEQSPACE - rcv_base + seqnum;
    
    bufferIndex = relativeSeq;
    
    
    if (!received[bufferIndex]) {

      unique_packets_received++;  
      printf("----B: First time receiving packet %d\n", seqnum);
    } else {
      retransmitted_packets_received++;  
      printf("----B: Received retransmission of packet %d\n", seqnum);
    }
  
      packets_received++;
      
      /* Calculate the buffer index properly considering wraparound */
      if (seqnum >= rcv_base)
        relativeSeq = seqnum - rcv_base;
      else
        relativeSeq = SEQSPACE - rcv_base + seqnum;
      
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
        while (next < WINDOWSIZE && received[next]) {
          /* Deliver this buffered packet */
          tolayer5(B, rcv_buffer[next].payload);
          
          /* Update state */
          received[next] = false;
          expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
          rcv_base = expectedseqnum;
          next++;
        }
        
        /* Shift the received array */
        for (i = 0; i < WINDOWSIZE - next; i++) {
          received[i] = received[i + next];
        }
        for (i = WINDOWSIZE - next; i < WINDOWSIZE; i++) {
          received[i] = false;
        }
      }
      
      /* Prepare SACK information - include all received but not yet delivered packets */
      sack.count = 0;
      for (i = 0; i < WINDOWSIZE; i++) {
        if (received[i] && ((rcv_base + i) % SEQSPACE != seqnum)) {
          /* Add to SACK list if different from primary ACK */
          sack.seqnums[sack.count++] = (rcv_base + i) % SEQSPACE;
          if (sack.count >= MAX_SACK) break;
        }
      }
    } else {
      bool is_old_packet = false;
      
      if (rcv_base > SEQSPACE/2) {
        if (seqnum < rcv_base && seqnum > (rcv_base + WINDOWSIZE) % SEQSPACE) {
          is_old_packet = true; 
        }
      } else {
        if (seqnum < rcv_base) {
          is_old_packet = true; 
        }
      }
      
      if (is_old_packet) {
        if (TRACE > 0) 
          printf("----B: old/duplicate packet %d received, send ACK\n", seqnum);
        sendpkt.acknum = seqnum;  
      } else {
        if (TRACE > 0) 
          printf("----B: future packet %d received, outside window\n", seqnum);
        sendpkt.acknum = NOTINUSE; 
      }
      
      /* Still include SACK information */
      sack.count = 0;
      for (i = 0; i < WINDOWSIZE; i++) {
        if (received[i]) {
          sack.seqnums[sack.count++] = (rcv_base + i) % SEQSPACE;
          if (sack.count >= MAX_SACK) break;
        }
      }
    }
  } else {
    /* packet is corrupted */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    sendpkt.acknum = NOTINUSE;
    
    /* Still include SACK information even if primary ACK is not useful */
    sack.count = 0;
    for (i = 0; i < WINDOWSIZE; i++) {
      if (received[i]) {
        sack.seqnums[sack.count++] = (rcv_base + i) % SEQSPACE;
        if (sack.count >= MAX_SACK) break;
      }
    }
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* Encode SACK information in payload */
  EncodeSACK(&sendpkt, sack);

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