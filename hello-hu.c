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
#include <unistd.h>
#include <sys/utsname.h>
#include <sys/time.h>
#include <stdlib.h>
#include <setjmp.h>

#include <shmem.h>
#define MASK_LOCK  0x00000080
#define MASK_PE  0x0000007F
#define MASK_VER 0xFFFFFF00
typedef struct {
  unsigned long pe:7;
  unsigned long l:1;
  unsigned long v:24;
} vlock_t;
long pSync[_SHMEM_REDUCE_SYNC_SIZE];
long pWrk[_SHMEM_REDUCE_SYNC_SIZE];

vlock_t lock[2]={0};
vlock_t tmlock[2]={0};
long account[2]={0};
long shared = 0;
long tm1, tm2, tm3;
//these codes are modified from tinySTM
typedef struct stm_tx {
  sigjmp_buf env;
  int me, npes;  //for SHMEM
  vlock_t lock[2];  //read set or write set
  long tmp_account[2];
  //for debug
  int commits;
  int aborts[4];
} stm_tx_t;
static stm_tx_t g_tx;  //global transactional descriptor
void stm_init(stm_tx_t* tx) {
  tx->me = shmem_my_pe ();
  tx->npes = shmem_n_pes();
}
void stm_start(stm_tx_t* tx) {
}
//find the lock corresponding to addr.
long stm_load(stm_tx_t* tx, long* addr) {
  return *addr;
}
//write the val into temp variable.
void stm_store(stm_tx_t* tx, long* addr, long val) {
  int i = 0;
  i = (int)(addr-account);
  tx->tmp_account[i] = val;
}

int check_lock(vlock_t* lock, int size) {
  int i;
  for(i=0; i<size; ++i) {
    if(lock[i].l)
      return 1;   // at least one locked in group
  }
  return 0;
}

int check_lv_all(vlock_t* target, vlock_t* source, int nsize, int PEsize) {
  int pe, ret = 0, i;
  vlock_t* pWrk;
  for(i=0; i<nsize; ++i)
    target[i] = source[i];
  pWrk = (vlock_t*)malloc(nsize * sizeof(vlock_t));
  for(pe=0; pe<PEsize; ++pe) {
    if(pe != shmem_my_pe()) {
      shmem_getmem(pWrk, source, nsize*sizeof(vlock_t), pe);
      if(check_lock(pWrk, nsize)){
	ret = 1;
	break;
      }
      if(pWrk[0].v > target[0].v || pWrk[1].v > target[1].v) {
        ret = 2; //old version
	for(i=0;i<nsize;++i)
	  target[i] = pWrk[i];
	//break;
      }
    }
  }

  free(pWrk);
  return ret;
}

void stm_commit(stm_tx_t* tx) {
  int tmp;
  tmp =check_lv_all(tmlock, lock, 2, tx->npes);
  if(tmp == 1) {
    tx->aborts[0] += 1;
    siglongjmp(tx->env, 0);  //what's the second param?
  }
  else if(tmp == 2) {
    tx->aborts[1] += 1;
    //synchronization
    lock[0].v = tmlock[0].v;
    lock[1].v = tmlock[1].v;
    shmem_getmem(account, account, 2*sizeof(long), tmlock[0].pe);
    siglongjmp(tx->env, 0);
  }
  lock[0].l = 1;//MASK_LOCK;
  lock[1].l = 1;//MASK_LOCK;
  __sync_synchronize();
  tmp = check_lv_all(tmlock, lock, 2, tx->npes);
  if(tmp == 1) {
    tx->aborts[2] += 1;
    lock[0].l = 0;
    lock[1].l = 0;
    siglongjmp(tx->env, 0);
  }
  else if(tmp == 2) {
    tx->aborts[3] += 1;
    lock[0].l = 0;//(ver[0]<<8)+me;
    lock[1].l = 0;//(ver[1]<<8)+me; //unlock
    lock[0].v = tmlock[0].v;
    lock[1].v = tmlock[1].v;
    shmem_getmem(account, account, 2*sizeof(long), tmlock[0].pe);
    siglongjmp(tx->env, 0);
  } 
  account[0] = tx->tmp_account[0];
  account[1] = tx->tmp_account[1];
  lock[0].v += 1; lock[1].v += 1;
  lock[0].l = lock[1].l = 0;
  tx->commits += 1;
}

static int transfer(long* src, long* dst, long amount) {
  long tm;
  stm_start(&g_tx);
  sigsetjmp(g_tx.env, 0);
  tm = stm_load(&g_tx, src);
  tm -= amount;
  stm_store(&g_tx, src, tm);
  tm = stm_load(&g_tx, dst);
  tm += amount;
  stm_store(&g_tx, dst, tm);
  stm_commit(&g_tx);
  return 0;
}

int
main (int argc, char **argv)
{
  int flag = 0; //transfer from A to B
  int overflow[2] = {0}; //control version overflow
  int me, npes, i=0, tm3;
  int commits = 0, aborts[4] = {0}, lock1=0;
  unsigned long ver[2] = {0};
  unsigned long tm1, tm2;
  struct utsname u;
  struct timeval start, end;
  int dur;

  for (i = 0; i < _SHMEM_REDUCE_SYNC_SIZE; i += 1) {
      pSync[i] = _SHMEM_SYNC_VALUE;
  }
  uname (&u);

  shmem_init ();

  stm_init(&g_tx);
  me = shmem_my_pe ();
  npes = shmem_n_pes ();
  lock[0].pe = me; lock[1].pe = me;

  gettimeofday(&start, NULL);
  while(1) {
    transfer(&account[0], &account[1], 1);

    gettimeofday(&end, NULL);
    dur = (end.tv_sec * 1000 + end.tv_usec / 1000) - (start.tv_sec * 1000 + start.tv_usec / 1000);
    if(dur > 1000)
      break;
  }
  //  shmem_barrier_all();
  printf("%s the %d of %d\n", u.nodename, me, npes);
  if((account[0]+account[1])==0) {
    printf ("verification passed! %d, %d\n", account[0], account[1]);
    printf("dur: %d, commits: %d, aborts: %d, %d, %d, %d\n",dur, g_tx.commits, g_tx.aborts[0], g_tx.aborts[1], g_tx.aborts[2], g_tx.aborts[3]);
    printf("ver1: %d,%d,%d ver2: %x \n", lock[0].v,lock[0].l,lock[0].pe, lock[0]);
  }
  else
    printf("verification failed, %d, %d\n", account[0], account[1]);

  return 0;
}
