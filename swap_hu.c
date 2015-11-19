/*
 *
 * Copyright (c) 2011 - 2014
 *   University of Houston System and Oak Ridge National Laboratory.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * 
 * o Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 
 * o Neither the name of the University of Houston System, Oak Ridge
 *   National Laboratory nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific
 *   prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



#include <stdio.h>

//#include <shmem.h>
#include <gasnet.h>
#include <stdlib.h>
#include <sys/utsname.h>
#include <sys/time.h>
  static int race_winner = -1;
#define LOAD_STORE_FENCE   __sync_synchronize
enum {
  GASNET_HANDLER_SWAP_OUT = 128,
  GASNET_HANDLER_SWAP_BAK
};
/**
 * NB we make the cond/value "long long" throughout
 * to be used by smaller types as self-contained payload
 */
gasnet_hsl_t amo_swap_lock=GASNET_HSL_INITIALIZER;
typedef struct
{
    void *local_store;          /* sender saves here */
    void *r_symm_addr;          /* recipient symmetric var */
    volatile int completed;     /* transaction end marker */
    volatile int *completed_addr;   /* addr of marker */
    size_t nbytes;              /* how big the value is */
    long long value;            /* value to be swapped */
    long long cond;             /* conditional value */
} atomic_payload_t;

static void handler_swap_bak(gasnet_token_t token, void* buf, size_t bufsiz) {
    atomic_payload_t *pp = (atomic_payload_t *) buf;

    gasnet_hsl_lock (&amo_swap_lock);
    //__transaction_atomic {
    /* save returned value */
      //    (void) memmove (pp->local_store, &(pp->value), pp->nbytes);
    (void) memmove (&race_winner, &(pp->value), pp->nbytes);

    //    LOAD_STORE_FENCE ();

    /* done it */
    *(pp->completed_addr) = 1;
    //}
    gasnet_hsl_unlock (&amo_swap_lock);
}
static void handler_swap_out(gasnet_token_t token, void* buf, size_t bufsiz) {
    long long old;
    atomic_payload_t *pp = (atomic_payload_t *) buf;

    gasnet_hsl_lock (&amo_swap_lock);
    //__transaction_atomic {

    /* save and update */
    (void) memmove (&old, &race_winner, pp->nbytes);
    (void) memmove (&race_winner, &(pp->value), pp->nbytes);
    pp->value = old;

    //LOAD_STORE_FENCE ();

    gasnet_hsl_unlock (&amo_swap_lock);
    //}

    /* return updated payload */
    gasnet_AMReplyMedium0 (token, GASNET_HANDLER_SWAP_BAK, buf, bufsiz);
}

static void make_swap_request(void* target, void* value, size_t nbytes, int pe, void* retval) {
    atomic_payload_t *p = (atomic_payload_t *) malloc (sizeof (*p));
    /*if (EXPR_UNLIKELY (p == (atomic_payload_t *) NULL)) {
        comms_bailout
            ("internal error: unable to allocate remote add payload memory");
	    }*/
    /* build payload to send */
    p->r_symm_addr = target;//shmemi_symmetric_addr_lookup (target, pe);
    p->nbytes = nbytes;
    p->value = *(long long *) value;
    p->completed = 0;
    p->completed_addr = &(p->completed);
    /* fire off request */
    gasnet_AMRequestMedium0 (pe, GASNET_HANDLER_SWAP_OUT, p, sizeof (*p));
    //GASNET_BLOCKUNTIL(p->completed);
    free (p);
}
static gasnet_handlerentry_t handlers[] = {
  {GASNET_HANDLER_SWAP_OUT, handler_swap_out},
  {GASNET_HANDLER_SWAP_BAK, handler_swap_bak}
};
int
main (int argc, char **argv)
{

  struct timeval start, end;
  int oldval, dur, tmp;
  int me, npes, i;
  struct utsname u;

  gasnet_node_t rank, size;
  gasnet_init(&argc, &argv);
  rank = gasnet_mynode();
  size = gasnet_nodes();
  gasnet_attach(handlers, 2, GASNET_PAGESIZE, GASNET_PAGESIZE);
  uname(&u);
  tmp = rank;
  //  start_pes (0);
  //me = shmem_my_pe ();
  //npes = shmem_n_pes();
  //shmem_barrier_all();
  gettimeofday(&start, NULL);
  //  if(rank == 0)
    for(i=1;i<size;i+=1)
      if(rank != i)
      make_swap_request (&race_winner, &tmp, sizeof(int), i, &oldval);

  /*if (oldval == -1)
    {
      printf ("pe %d was first\n", me);
      }*/
  gettimeofday(&end, NULL);
  dur = (end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec);
  printf("%s pe %d/%d spent %d us %d, %d\n", u.nodename, rank, size, dur, oldval, race_winner);
  gasnet_barrier_notify(0,GASNET_BARRIERFLAG_ANONYMOUS);
  gasnet_barrier_wait(0,GASNET_BARRIERFLAG_ANONYMOUS);
  gasnet_exit(0);
  return 0;
}
