/*
 * Minimal selftest binary for app/build pack.
 * Build with: make
 */

 #include <stdio.h>
 #include <sys/utsname.h>
 #include <unistd.h>
 
 int main(void) {
   struct utsname u;
   uname(&u);
 
   printf("SELFTEST: arch=%s sys=%s release=%s version=%s\n",
		  u.machine, u.sysname, u.release, u.version);
 
   printf("SELFTEST: uid=%d pid=%d\n", (int)getuid(), (int)getpid());
 
   // This is the CI marker
   printf("APP_OK\n");
   return 0;
 }