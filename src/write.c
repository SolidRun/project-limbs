#include <stdio.h>
#include <unistd.h>
#include <time.h>

void print_help (void) {
	printf ("rwv <file name> <stm32 ttyACM>\n");
}

int main(int argc, char **argv) {
	FILE *fin, *stm;
	int addr = 0, addr2 = 0;
	int len = 0, mark_break = 0, i;
	unsigned char buffer[256];
	if (argc != 3) {
		print_help();
		return 0;
	}

	fin = fopen (argv[1], "r");
	if (!fin) {
		printf ("Can't open %s\n",argv[1]);
		return -1;
	}
	stm = fopen (argv[2], "a+");
	if (!stm) {
		printf ("Can't open STM ACM %s\n",argv[2]);
		return -1;
	}
	len = 0;
	while (1) {
		int input = fgetc(fin);
		if (input == EOF) mark_break = 1;
		else {
			buffer[len] = (unsigned char) input;
			len ++;
		}
		if (mark_break && !len) break;
		if ((len == 256) || (mark_break)) {
			char str[550];
			sprintf (str, "sp 0x%08x 0x",addr & 0xffffff00);
			for (i = 0 ; i < len ; i++) {
				sprintf (&str[8+8+i*2],"%02x",buffer[i]);
			}
			printf ("%s\n",str);
			len = 0;
			fprintf(stm, "%s\n",str);
			fflush(stm);
		}
		if (mark_break) break;
		addr ++;
	}
	fclose (stm);
	fclose (fin);
}
