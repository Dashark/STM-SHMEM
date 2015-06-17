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



/*
 * reduce [0,1,2] + shmem_my_pe() across 4 PEs with MAX()
 *
 *
 */

#include <stdio.h>
#include <string.h>

#include <mpp/shmem.h>

long pSync[_SHMEM_REDUCE_SYNC_SIZE];
long pWrk[_SHMEM_REDUCE_SYNC_SIZE];

#define N 3

unsigned long src[N];
unsigned long dst[N];

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
    shmem_barrier (PE_start, logPE_stride, PE_size, pSync);             
    /* now go through other PEs and get source */                       
    pe = PE_start;                                                      
    for (i = 0; i < PE_size; i+= 1) 
      {                                                                 
        if (mype != pe)                                     
          {                                                             
            int k;                                                      
            int ti = 0, si = 0; /* target and source index walk */      
            /* pull in all the full chunks */                           
	    /*      nget = _SHMEM_REDUCE_MIN_WRKDATA_SIZE * sizeof (long);      
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
    /*            shmem_getmem (pWrk, & (source[si]), nget, pe);              
            for (j = 0; j < nrem; j += 1)                               
              {                                                         
                write_to[ti] = write_to[ti]>pWrk[j]?write_to[ti]:pWrk[j];
                ti += 1;                                                
		}*/
	    printf("mype:%d, pe:%d, nloops: %d\n", mype, pe, _SHMEM_REDUCE_MIN_WRKDATA_SIZE);                        
          } 
        pe += step;                                                     
      }                                                                 
    /* everyone has to have finished */                                 
    //shmem_barrier (PE_start, logPE_stride, PE_size, pSync);
}
void
shmem_min_to_all_nobarrier(long* target, long* source, int nreduce,	
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
                    write_to[ti] = write_to[ti]<pWrk[j]?write_to[ti]:pWrk[j];
                    ti += 1;                                            
                  }                                                     
                si += _SHMEM_REDUCE_MIN_WRKDATA_SIZE;                   
              }                                                         
            nget = nrem * sizeof (long);                                
            /* now get remaining part of source */                      
            shmem_getmem (pWrk, & (source[si]), nget, pe);              
            for (j = 0; j < nrem; j += 1)                               
              {                                                         
                write_to[ti] = write_to[ti]<pWrk[j]?write_to[ti]:pWrk[j];
                ti += 1;                                                
              }                                                         
          }                                                             
        pe += step;                                                     
      }                                                                 
    /* everyone has to have finished */                                 
    //shmem_barrier (PE_start, logPE_stride, PE_size, pSync);
}

int
main ()
{
  int i, npes;

  for (i = 0; i < _SHMEM_REDUCE_SYNC_SIZE; i += 1)
    {
      pSync[i] = _SHMEM_SYNC_VALUE;
    }

  start_pes (0);
  npes = shmem_n_pes();

  for (i = 0; i < N; i += 1)
    {
      src[i] = shmem_my_pe () + i;
      //src[i] |= 0x80000000;
    }
  //  if(shmem_my_pe() == 0) src[0] = 0xfffffff0;
  shmem_barrier_all ();

  if(shmem_my_pe() == 0)
    shmem_getmem (pWrk, src, 4, 1);              
      shmem_long_max_to_all(dst, src, 3, 0, 0, npes, pWrk, pSync);
  //      shmem_max_to_all_nobarrier (dst, src, 3, 0, 0, npes, pWrk, pSync);
    //  else
    //    shmem_min_to_all_nobarrier (dst, src, 3, 0, 0, npes, pWrk, pSync);
  
  printf ("%d/%d   dst =", shmem_my_pe (), shmem_n_pes ());
  for (i = 0; i < N; i += 1)
    {
      printf (" %d", dst[i]);
    }
  printf("  %d", pWrk[0]);
  printf ("\n");

  return 0;
}
