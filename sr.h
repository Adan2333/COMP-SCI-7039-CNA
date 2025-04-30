/* Header file for Selective Repeat Protocol */
#ifndef SR_H
#define SR_H


/* Debug Variables */
extern int TRACE;
extern int YES;
extern int NO;

/* Sender (A) function prototypes */
void A_output(struct msg message);
void A_input(struct pkt packet);
void A_timerinterrupt(void);
void A_init(void);

/* Receiver (B) function prototypes */
void B_input(struct pkt packet);
void B_init(void);
void B_output(struct msg message);
void B_timerinterrupt(void);

/* Global statistics counters */
extern int packets_resent;
extern int packets_received;
extern int window_full;
extern int total_ACKs_received;
extern int new_ACKs;

#ifndef BIDIRECTIONAL
#define BIDIRECTIONAL 0
#endif