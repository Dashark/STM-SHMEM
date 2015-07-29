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

typedef struct {
  unsigned long pe:7;
  unsigned long l:1;
  unsigned long v:24;
} vlock_t;
long pSync[_SHMEM_REDUCE_SYNC_SIZE];
long pWrk[_SHMEM_REDUCE_SYNC_SIZE];

vlock_t lock[1024]={0};
vlock_t tmlock[1024]={0};
long account[1024]={0};


//these codes are modified from tinySTM
typedef struct w_entry {
  long* addr;
  long value;
  vlock_t* lock;
} w_entry_t;

typedef struct w_set {
  w_entry_t entries[1024];
  unsigned long nb; //number of entries
} w_set_t;

typedef struct stm_tx {
  sigjmp_buf env;
  int me, npes;  //for SHMEM

  w_set_t w_set;
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
  tx->w_set.nb = 0;
}
//find the lock corresponding to addr.
long stm_load(stm_tx_t* tx, long* addr) {
  return *addr;
}
//write the val into temp variable.
void stm_store(stm_tx_t* tx, long* addr, long val) {
  int i = 0;
  for(i = 0; i < tx->w_set.nb; i+=1)
    if(addr == tx->w_set.entries[i].addr) {
      tx->w_set.entries[i].value = val;
      return;
    }

  tx->w_set.entries[tx->w_set.nb].addr = addr;
  tx->w_set.entries[tx->w_set.nb].lock = &lock[addr-account];
  tx->w_set.entries[tx->w_set.nb].value = val;
  tx->w_set.nb += 1;
}

int check_all(stm_tx_t* tx) {
  int pe, ret = 0, i;
  vlock_t pWrk[1024], max_lv[1024]={0};
  for(i=0;i<1024;i+=1)
    max_lv[i] = lock[i];

  for(pe=0; pe<tx->npes; ++pe) {
    if(pe != tx->me) {
      shmem_getmem(pWrk, lock, 1024*sizeof(vlock_t), pe);
      for(i=0; i<tx->w_set.nb; i+=1) {
        int idx = (int)(tx->w_set.entries[i].addr - account);
	if(pWrk[idx].l == 1) {
	  ret = 1;
	  return ret;    // could be continue?
	}
	else if(pWrk[idx].v > max_lv[idx].v) {
	  max_lv[idx] = pWrk[idx];  //may not be maximum
	  ret = 2;
	}
      }
    }
  }
  if(ret == 2) {
    for(i=0;i<tx->w_set.nb; i+=1) {
      int idx = (int)(tx->w_set.entries[i].addr - account);
      shmem_getmem(tx->w_set.entries[i].addr, tx->w_set.entries[i].addr, sizeof(long), max_lv[idx].pe);
      lock[idx].v = max_lv[idx].v;
    }
  }
  return ret;
}

void lock_all(stm_tx_t* tx, int flag) {
  int i = 0;
  for(i = 0; i < tx->w_set.nb; i+= 1)
    tx->w_set.entries[i].lock->l = flag==0?0:1;
}
void stm_commit(stm_tx_t* tx) {
  int ret = 0, i;
  ret = check_all(tx);
  if(ret == 1) {
    tx->aborts[0] += 1;
    tx->w_set.nb = 0;
    siglongjmp(tx->env, 0);
  }
  else if(ret == 2) {
    tx->aborts[1] += 1;
    tx->w_set.nb = 0;
    siglongjmp(tx->env, 0);
  }
  lock_all(tx, 1);
  __sync_synchronize();
  ret = check_all(tx);
  if(ret == 1) {
    tx->aborts[2] += 1;
    lock_all(tx, 0);
    tx->w_set.nb = 0;
    siglongjmp(tx->env, 0);
  }
  else if(ret == 2) {
    tx->aborts[3] += 1;
    lock_all(tx, 0);
    tx->w_set.nb = 0;
    siglongjmp(tx->env, 0);
  }
  for(i = 0; i < tx->w_set.nb; i += 1) {
    *(tx->w_set.entries[i].addr) = tx->w_set.entries[i].value;
    tx->w_set.entries[i].lock->v += 1;
  }
  lock_all(tx, 0);
  tx->commits += 1;
}
static int transfer(long* src, long* dst, long amount) {
  long tm;

  stm_start(&g_tx);
  sigsetjmp(g_tx.env, 0);
  //    if(g_tx.aborts[0]>1000||g_tx.aborts[1]>1000||g_tx.aborts[2]>1000||g_tx.aborts[3]>1000) {
      //printf("infinite aborts\n");
  //    return 0;
  //  }
  tm = stm_load(&g_tx, src);
  tm -= amount;
  stm_store(&g_tx, src, tm);
  tm = stm_load(&g_tx, dst);
  tm += amount;
  stm_store(&g_tx, dst, tm);
  stm_commit(&g_tx);
  return 0;
}

int verify1(long* acc, int size) {
  int i, ret = 0;
  for(i=0;i<size;i+=1)
    ret += acc[i];
  return ret;
}
int verify(long* acc, int size) {
  int i,j,k, ret = 0;
  for(j = 0; j < g_tx.npes; j += 1) {
    if(j != g_tx.me) {
      shmem_getmem(tmlock, lock, 1024*sizeof(vlock_t), j);
      for(k = 0; k < 1024; k += 1)
	if(tmlock[k].v > lock[k].v)
	  lock[k] = tmlock[k];
    }
  }
  for(i=0; i<1024; i+=1)
    if(lock[i].pe == g_tx.me)
      ret += acc[i];
  return ret;
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

  int src, dst;
  int rand_max, rand_min, rsd[3];
  unsigned short seed[3];
  rand_max = 1023; rand_min = 0;
  rsd[0] = rand(); rsd[1] = rand(); rsd[2] = rand();
  seed[0] = (unsigned short)rand_r(&rsd[0]);
  seed[1] = (unsigned short)rand_r(&rsd[0]);
  seed[2] = (unsigned short)rand_r(&rsd[0]);
  for (i = 0; i < _SHMEM_REDUCE_SYNC_SIZE; i += 1) {
      pSync[i] = _SHMEM_SYNC_VALUE;
  }
  uname (&u);

  shmem_init ();

  stm_init(&g_tx);
  me = shmem_my_pe ();
  npes = shmem_n_pes ();
  for(i=0;i<1024;i+=1)
    lock[i].pe = me;
  tm2 = verify1(account, 1024);
  gettimeofday(&start, NULL);
  while(1) {
    src = (int)(erand48(seed) * rand_max) + rand_min;
    dst = (int)(erand48(seed) * rand_max) + rand_min;
    if(dst == src)
      dst = (src+1)%rand_max + rand_min;
    transfer(&account[src], &account[dst], 1);

    gettimeofday(&end, NULL);
    dur = (end.tv_sec * 1000 + end.tv_usec / 1000) - (start.tv_sec * 1000 + start.tv_usec / 1000);
    if(dur > 10000)
      break;
  }
  shmem_barrier_all();
  printf("%s the %d of %d\n", u.nodename, me, npes);
  tm1 = verify1(account, 1024);
  tm3 = verify(account, 1024);
  printf ("verification passed? %d, %d, %d\n", tm1, tm2, tm3);
    printf("dur: %d, commits: %d, aborts: %d, %d, %d, %d\n",dur, g_tx.commits, g_tx.aborts[0], g_tx.aborts[1], g_tx.aborts[2], g_tx.aborts[3]);

  return 0;
}
