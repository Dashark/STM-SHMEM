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

#include <shmem.h>
#define MASK_LOCK  0x00000080
#define MASK_PE  0x0000007F
#define MASK_VER 0xFFFFFF00

long pSync[_SHMEM_REDUCE_SYNC_SIZE];
long pWrk[_SHMEM_REDUCE_SYNC_SIZE];

unsigned long lock[2]={0};
unsigned long tmlock[2]={0};
unsigned long account[2]={0};
long shared = 0;
long tm1, tm2, tm3;

void
shmem_max_to_all_nobarrier(long* target, long* source, int nreduce,	
                               int PE_start, int logPE_stride, int PE_size, 
                               long *pWrk, long *pSync)                 
  {                                                                     
    const int step = 1 << logPE_stride;                                 
    const int nloops = nreduce / _SHMEM_REDUCE_MIN_WRKDATA_SIZE;        
    const int nrem = nreduce % _SHMEM_REDUCE_MIN_WRKDATA_SIZE;          
    const int snred = sizeof(long) * nreduce;                           

    size_t nget;                                                        
    int i, j;                                                           
    int pe, mype;                                                             
    long* tmptrg = NULL;                                                
    long* write_to=target;                                               
    mype = shmem_my_pe();
    /* everyone must initialize */                                      
    for (j = 0; j < nreduce; j += 1) {                                                 write_to[j] = source[j];                                        
    }                                                                 
    //shmem_barrier (PE_start, logPE_stride, PE_size, pSync);             
    /* now go through other PEs and get source */                       
    pe = PE_start;                                                      
    for (i = 0; i < PE_size; i+= 1) 
      {                                                                 
        if (mype != pe)                                     
          {                                                             
            int k;                                                      
            int ti = 0, si = 0; /* target and source index walk */      
            /* pull in all the full chunks */                           
            nget = _SHMEM_REDUCE_MIN_WRKDATA_SIZE * sizeof (long);      
            for (k = 0; k < nloops; k += 1)                             
              {                                                         
                shmem_getmem (pWrk, & (source[si]), nget, pe);          
                for (j = 0; j < _SHMEM_REDUCE_MIN_WRKDATA_SIZE; j += 1)	
                  {                                                     
                    write_to[ti] = write_to[ti]>pWrk[j]?write_to[ti]:pWrk[j];
                    ti += 1;                                            
                  }                                                     
                si += _SHMEM_REDUCE_MIN_WRKDATA_SIZE;                   
              }                                                         
            nget = nrem * sizeof (long);                                
            /* now get remaining part of source */                      
            shmem_getmem (pWrk, & (source[si]), nget, pe);              
            for (j = 0; j < nrem; j += 1)                               
              {                                                         
                write_to[ti] = write_to[ti]>pWrk[j]?write_to[ti]:pWrk[j];
                ti += 1;                                                
              }                                                         
          }                                                             
        pe += step;                                                     
      }                                                                 
    /* everyone has to have finished */                                 
    //shmem_barrier (PE_start, logPE_stride, PE_size, pSync);
}

int check_lv() {
  int npes;
  unsigned long ver[2] = {0}, tmver[2]={0};
  npes = shmem_n_pes();
  ver[0] = lock[0]>>8; ver[1] = lock[1]>>8; //keep the version
  shmem_max_to_all_nobarrier(tmlock, lock, 2, 0, 0, npes, pWrk, pSync);
  tmver[0] = tmlock[0] >> 8;
  tmver[1] = tmlock[1] >> 8;
  if((tmver[0] > ver[0]) || (tmver[1] > ver[1]) || tmlock[0]&MASK_LOCK || tmlock[1]&MASK_LOCK) { //already locked or not the newest, aborting
    return 1;
  }
  return 0;
}

void sync_lv() {
  int npes;
  unsigned long tm3 = -1;
  npes = shmem_n_pes();
  shmem_max_to_all_nobarrier(tmlock, lock, 2, 0, 0, npes, pWrk, pSync);
  if(((tmlock[0]&MASK_LOCK) == 0) && ((tmlock[1]&MASK_LOCK) == 0)) {
    tm3 = tmlock[0]&MASK_PE;
    if(tm3 > npes || tm3 < 0)
      return ;
    shmem_long_get(account, account, 2, tm3);
    shmem_long_get(tmlock, lock, 2, tm3);
    lock[0] = tmlock[0]&MASK_VER+lock[0]&MASK_PE;
    lock[1] = tmlock[1]&MASK_VER+lock[1]&MASK_PE;
  }
}

int
main (int argc, char **argv)
{
  int flag = 0; //transfer from A to B
  int overflow[2] = {0}; //control version overflow
  int me, npes, i=0;
  int commits = 0, aborts = 0, lock1=0;
  unsigned long ver[2] = {0};
  unsigned long tm1, tm2;
  struct utsname u;

  for (i = 0; i < _SHMEM_REDUCE_SYNC_SIZE; i += 1) {
      pSync[i] = _SHMEM_SYNC_VALUE;
  }
  uname (&u);

  shmem_init ();

  me = shmem_my_pe ();
  npes = shmem_n_pes ();
  lock[0] = me; lock[1] = me;

  while(1) {
    //for debuging
    if(aborts > 100000)
      break;
    //stm_begin();
    //stm_read();
    tm1 = account[0];
    tm2 = account[1];
    //stm_write();
    //    if(flag == 0) {
      tm1 = tm1 - 1;
      tm2 = tm2 + 1;
      //    }
      //    else {
      //      tm1 = tm1 + 1;
      //      tm2 = tm2 - 1;
      //    }
    //commit
    ver[0] = lock[0]>>8; ver[1] = lock[1]>>8; //keep the version
    if((ver[0]+1)>=(1<<24))
      overflow[0] = 1;
    if((ver[1]+1)>=(1<<24))
      overflow[1] = 1;
    // is the newest version?
    if(check_lv()) {
      flag = flag == 0?1:0;  //change transferbound
      aborts += 1;
      //synchronization
      sync_lv();
      continue;
    }
    //locking
    lock[0] += me + MASK_LOCK;
    lock[1] += me + MASK_LOCK;
    // is the newest version? because no atomic
    shmem_max_to_all_nobarrier(tmlock, lock, 2, 0, 0, npes, pWrk, pSync);
    if(tmlock[0] != lock[0] || tmlock[1] != lock[1]) {
      flag = flag == 0?1:0;
      aborts += 1;
      lock[0] = (ver[0]<<8)+me; lock[1] = (ver[1]<<8)+me; //unlock
      continue;
    } 
    account[0] = tm1;
    account[1] = tm2;
    ver[0] += 1; ver[1] += 1;
    //version overflow
    if(overflow[0]) {
      ver[0] = 1;
      //shmem_broadcast
      for(i = 0; i < npes; ++i) {
	if(i != me)
          shmem_long_p(&lock[0], -1, i);
      }
    }
    if(overflow[1]) {
      ver[1] = 1;
      for(i = 0; i < npes; ++i) {
	if(i != me)
	  shmem_long_p(&lock[1], -1, i);
      }
    }
    lock[0] = (ver[0]<<8)+me; lock[1] = (ver[1]<<8)+me; //unlock and update version.
    if(commits++ > 1000)
      break;
  }
  //  shmem_barrier_all();
  printf("%s the %d of %d\n", u.nodename, me, npes);
  if((account[0]+account[1])==0) {
    printf ("verification passed! %d, %d\n", account[0], account[1]);
    printf("commits: %d, aborts: %d\n", commits, aborts);
    printf("ver1: %d, ver2: %x \n", lock[0]>>8, lock[1]);
  }
  else
    printf("verification failed, %d, %d\n", account[0], account[1]);

  return 0;
}
