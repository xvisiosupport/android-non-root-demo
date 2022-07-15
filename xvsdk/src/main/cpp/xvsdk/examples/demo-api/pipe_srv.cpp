

#ifdef _WIN32
int main()
{

}

#else
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "pipe_srv.h"

static int vsc_srv_recv_pipe_fd = -1;
static int vsc_srv_send_pipe_fd = -1;

#define MENU_INFO_LEN		(4096)
static char menu_info[MENU_INFO_LEN] = { 0 };


void vsc_srv_sig_handler(int sig)
{
	switch (sig) {
	case SIGINT:
		printf("catch SIGINT\n");
		break;
#ifdef SIGHUP
	case SIGHUP:
		printf("catch SIGHUB\n");
		break;
#endif
	case SIGTERM:
		printf("catch SIGTERM\n");
		break;
	}printf("%s\n", __func__);
	vsc_srv_release_handle(&vsc_srv_send_pipe_fd, &vsc_srv_recv_pipe_fd);
	exit(0);
}

int main(void)
{
	int retval;
	int cmdbuf[4] = { SRV_CMD_FLAG1, SRV_CMD_FLAG2, SRV_CMD_FLAG3, -1 };

	retval = vsc_pipe_init();
	if (retval != 0) {
		printf("vsc cmd pipe init fail: %d\n", retval);
		return retval;
	}

	printf("wait for client connect....\n");
	retval = vsc_server_get_handle(&vsc_srv_send_pipe_fd, &vsc_srv_recv_pipe_fd);
	if (retval != 0) {
		printf("vsc pipe server get handler fail: %d\n", retval);
		return retval;
	}
	signal(SIGINT, vsc_srv_sig_handler);

	printf("ready handle: send:%d,recv:%d\n", vsc_srv_send_pipe_fd, vsc_srv_recv_pipe_fd);
	while (1) {
		retval = read(vsc_srv_recv_pipe_fd, menu_info, sizeof(menu_info) - 2);
		menu_info[retval] = '\0';
		printf("%s", menu_info);
		//// only get input when have predefined header
		//if (strncmp(menu_info, "\n\nDemo\n", 6) == 0) {
		scanf("%d", &cmdbuf[3]);
		write(vsc_srv_send_pipe_fd, cmdbuf, sizeof(cmdbuf));
		//}
	}

	vsc_srv_release_handle(&vsc_srv_send_pipe_fd, &vsc_srv_recv_pipe_fd);
	return 0;
}

#endif