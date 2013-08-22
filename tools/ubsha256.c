/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Vijay Kumar Bhatt   b44340@freescale.com
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "os_support.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include "sha256.h"
#include "d4400image.h"

int main(int argc, char **argv)
{
	unsigned char output[SHA256_SUM_LEN];
	int i, len;

	char	*imagefile;
	char    *secure_uboot_file;
	char	*cmdname = *argv;
	unsigned char	*ptr;
	unsigned char	*data;
	struct stat sbuf;
	unsigned char	*ptroff;
	int	ifd;
	FILE    *ifd_secure;

	if (argc > 1) {
		imagefile = argv[1];
		secure_uboot_file = argv[2];
		ifd = open(imagefile, O_RDWR|O_BINARY);
		ifd_secure = fopen(secure_uboot_file, "wb+");
		if (ifd < 0) {
			fprintf(stderr, "%s: Can't open %s: %s\n",
				cmdname, imagefile, strerror(errno));
			exit(EXIT_FAILURE);
		}
		if (!ifd_secure) {
			fprintf(stderr, "%s: Can't open %s\n",
				cmdname, strerror(errno));
			exit(EXIT_FAILURE);
		}
		if (fstat(ifd, &sbuf) < 0) {
			fprintf(stderr, "%s: Can't stat %s: %s\n",
				cmdname, imagefile, strerror(errno));
			exit(EXIT_FAILURE);
		}
		len = sbuf.st_size;
		ptr = (unsigned char *)mmap(0, len,
				PROT_READ, MAP_SHARED, ifd, 0);
		if (ptr == (unsigned char *)MAP_FAILED) {
			fprintf(stderr, "%s: Can't read %s: %s\n",
				cmdname, imagefile, strerror(errno));
			exit(EXIT_FAILURE);
		}

		/* create a copy, so we can blank out the sha256 sum */
		data = (unsigned char *)malloc(IVT_OFFSET_NOR +
				len + SHA256_SUM_LEN);
		memset(data, 0xff, (IVT_OFFSET_NOR + len + SHA256_SUM_LEN));
		memcpy((data + IVT_OFFSET_NOR), ptr, len);
		ptroff = &data[len + IVT_OFFSET_NOR];
		for (i = 0; i < SHA256_SUM_LEN; i++)
			ptroff[i] = 0;

		sha256_csum_wd((unsigned char *)data, (len + IVT_OFFSET_NOR),
			       (unsigned char *)output, CHUNKSZ_SHA256);
		printf("U-Boot sum:\n");
		for (i = 0; i < 32; i++)
			printf("%02X ", output[i]);

		printf("\n");
		/* Write the sum in the data buf */

		for (i = 0; i < 32; i++)
			ptroff[i] = output[i];

		printf("\n");
		if (fwrite((data + IVT_OFFSET_NOR), 1, (len + SHA256_SUM_LEN),
			   ifd_secure) != (len + SHA256_SUM_LEN)) {
			fprintf(stderr, "%s: Can't write  %s\n",
				cmdname, strerror(errno));
			exit(EXIT_FAILURE);
		}
		free(data);
		(void) munmap((void *)ptr, len);
		(void) close(ifd);
		(void) fclose(ifd_secure);
	}
	return EXIT_SUCCESS;
}
