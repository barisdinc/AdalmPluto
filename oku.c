#include <stdio.h>
#include <string.h>

typedef struct {
    unsigned int real, imag;
} Complex;

int main () {
   FILE *fp;
   Complex buffer[100];
   //char buffer[100];
   int x;
   fp = fopen("iq.dat", "r");
     fread(buffer, 8, 10, fp);
   for (x=0;x<20;x++)
   {
     //fseek(fp, x, SEEK_SET);
     printf("%d ", buffer[x].real);
     //printf("%d ", buffer[x]);
   
   }
   fclose(fp);
   
   return(0);
}

