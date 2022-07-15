#ifndef __VSC_CLIENT_H_
#define __VSC_CLIENT_H_

#ifdef _WIN32

#else
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>


#define VSC_SRV_RECV_PIPE	"./vscSrvRfifo"   // client write,
#define VSC_SRV_SEND_PIPE	"./vscSrvWfifo"   // client read

#define SRV_CMD_FLAG1	('C')
#define SRV_CMD_FLAG2	('M')
#define SRV_CMD_FLAG3	('D')

#define IS_PIPE_SRV_CMD(cmd1, cmd2, cmd3) \
	((cmd1==SRV_CMD_FLAG1) && (cmd2==SRV_CMD_FLAG2) && (cmd3==SRV_CMD_FLAG3))

static int vsc_pipe_init(void)
{
	int ret;

	if (access(VSC_SRV_RECV_PIPE, F_OK) != 0) {
		ret = mkfifo(VSC_SRV_RECV_PIPE, 0666);
		if (0 != ret) {
			printf("mkfifo server recv pipe fail, ret: %d\n", ret);
			return -1;
		}
		printf("create server recv pipe\n");
	} else {
		printf("pipe server recv has already exist\n");
	}

	if (access(VSC_SRV_SEND_PIPE, F_OK) != 0) {
		ret = mkfifo(VSC_SRV_SEND_PIPE, 0666);
		if (0 != ret) {
			printf("mkfifo server send pipe fail, ret: %d\n", ret);
			return -1;
		}
		printf("create server send pipe\n");
	} else {
		printf("pipe server send has already exist\n");
	}

	return 0;
}

/*--------------  server API -------------------------------------- */
static int vsc_server_get_handle(int *sendfd, int *recvfd)
{
	int ret = 0;

	ret = open(VSC_SRV_SEND_PIPE, O_WRONLY);
	if (ret < 0) {
		printf("open vsc cmd pipe for server fail, %d\n", ret);
		return ret;
	}
	*sendfd = ret;
	printf("pipe server get send handle: %d\n", *sendfd);

	ret = open(VSC_SRV_RECV_PIPE, O_RDONLY);
	if (ret < 0) {
		printf("open vsc cmd pipe for server fail, %d\n", ret);
		return ret;
	}
	*recvfd = ret;
	printf("pipe server get recv handle: %d\n", *recvfd);

	return 0;
}

static void vsc_srv_release_handle(int *sendfd, int *recvfd)
{
	if (*recvfd > 0)
		close(*recvfd);

	if (*sendfd > 0)
		close(*sendfd);
	return;
}
/*------------------------------------------------------------------------- */
static int vsc_cmd_recv_fd = -1;
static int vsc_cmd_send_fd = -1;
static int vsc_pipe_srv_pid = -1;

static int vsc_client_pipe_init(void)
{
	int ret = 0;

	ret = vsc_pipe_init();
	if (ret != 0) return ret;

	ret = open(VSC_SRV_SEND_PIPE, O_RDONLY);
	if (ret < 0) {
		printf("open vsc cmd pipe for client recv fail, %d\n", ret);
		return ret;
	}
	vsc_cmd_recv_fd = ret;
	printf("client pipe get recv handle: %d\n", vsc_cmd_recv_fd);

	ret = open(VSC_SRV_RECV_PIPE, O_WRONLY);
	if (ret < 0) {
		printf("open vsc cmd pipe for client send fail, %d\n", ret);
		return ret;
	}
	vsc_cmd_send_fd = ret;
	printf("client pipe get send handle:%d\n", vsc_cmd_send_fd);

	printf("command pipe init for client successful\n");
	return 0;
}

static void vsc_client_pipe_deinit(void)
{
	if (vsc_cmd_send_fd > 0)
		close(vsc_cmd_send_fd);

	if (vsc_cmd_recv_fd > 0)
		close(vsc_cmd_recv_fd);

	return;
}
static int vsc_client_pipe_get_srv_pid()
{
	FILE *fp = popen("ps -e | grep \'pipe_srv\' | awk \'{print $1}\'", "r");
	char buffer[10] = {0};

    if (fp == NULL) {
		printf("Cannot find pipe service process\n");
        return -1;
    }

	while (NULL != fgets(buffer, 10, fp)) {
		vsc_pipe_srv_pid = atoi(buffer);
		printf("vsc pipe server pid: %d\n", vsc_pipe_srv_pid);
	}
	pclose(fp);
	return 0;
}

static int vsc_client_pipe_request_cmd(const char *tip_info, int size)
{
	int cmdbuf[16] = {0};
	int retval;

	retval = write(vsc_cmd_send_fd, tip_info, size);
	retval = read(vsc_cmd_recv_fd, cmdbuf, sizeof(cmdbuf));
	if (IS_PIPE_SRV_CMD(cmdbuf[0], cmdbuf[1], cmdbuf[2])) {
		printf("recv cmd %d\n", cmdbuf[3]);
		return cmdbuf[3];
	}
	return -1;
}

static int vsc_client_pipe_terminal_srv(void)
{
	if (vsc_pipe_srv_pid > 0)
		kill(vsc_pipe_srv_pid, SIGTERM);
	return 0;
}
#endif

#endif //__VSC_CLIENT_H_
