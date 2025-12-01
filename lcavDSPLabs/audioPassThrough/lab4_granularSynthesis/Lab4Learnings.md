This lab was pretty challenging. I first had to wrap my head around granular synthesis, which took some time due to my
infamiliarity with time domain techniques. I first implemented the technique in python and then ported it over to C. The
porting to C was a bit painful, because of a hidden bug in the dithering function, where I used the asm() directive incorrectly
leading to weird behaviour when compiling with -O3. There was also a hidden narrowing error in my ring buffer implementation,
this took some time to hunt down because the compiler did not throw any warning due to narrowing. I will have to run static
analysis on my code manually from now on, it could have saved me a lot of hours.

Finally I am very happy with how the effect turned out, even if the road was not very easy. In the next lab we will try to make
the resulting sound effect sound a bit more natural. Stay tuned!