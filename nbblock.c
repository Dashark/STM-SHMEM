/*
 *
 * Copyright (c) 2011, 2012 
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
#include <sys/utsname.h>
#include <shmem.h>
static int locks[1024];
int
main (int argc, char **argv)
{
  int me, npes;
  struct utsname u;
  uname(&u);
  int rand_max, rand_min, rsd[3];
  unsigned short seed[3];
  rand_max = 1023; rand_min = 0;
  rsd[0] = rand(); rsd[1] = rand(); rsd[2] = rand();
  seed[0] = (unsigned short)rand_r(&rsd[0]);
  seed[1] = (unsigned short)rand_r(&rsd[0]);
  seed[2] = (unsigned short)rand_r(&rsd[0]);

  shmem_init();

  me = shmem_my_pe ();
  npes = shmem_n_pes ();

  src = (int)(erand48(seed) * rand_max) + rand_min;
  dst = (int)(erand48(seed) * rand_max) + rand_min;
  //lock self
  if(locks[src]==0) locks[src]=me;
  broadcast_lock_nb();
  //do self computing and don't wait for locking results
  broadcast_unlock_nb();
  //unlock self
  printf ("Hello %s from node %4d of %4d\n", u.nodename, me, npes);

  return 0;
}
