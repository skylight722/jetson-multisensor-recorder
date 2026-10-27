#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CDI_TSC_FSYNC _IOW('T', 1, int)
#define CDI_TSC_DEVICE "/dev/cdi_tsc"

static volatile sig_atomic_t stop_requested = 0;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static int install_signal_handlers(void)
{
    struct sigaction action;

    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    if (sigaction(SIGINT, &action, NULL) < 0) {
        return -1;
    }

    if (sigaction(SIGTERM, &action, NULL) < 0) {
        return -1;
    }

    if (sigaction(SIGHUP, &action, NULL) < 0) {
        return -1;
    }

    return 0;
}

int main(void)
{
    int fd = -1;
    int on = 1;
    int off = 0;
    int exit_status = EXIT_FAILURE;
    const bool interactive = isatty(STDIN_FILENO);

    if (install_signal_handlers() < 0) {
        perror("signal handler 설정 실패");
        return EXIT_FAILURE;
    }

    fd = open(CDI_TSC_DEVICE, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        perror("cdi_tsc open 실패");
        return EXIT_FAILURE;
    }

    printf("TSC Generator 시작 명령 전송...\n");

    if (ioctl(fd, CDI_TSC_FSYNC, &on) < 0) {
        perror("TSC Generator ON ioctl 실패");
        goto cleanup;
    }

    printf("30 Hz FSYNC 펄스 출력이 시작되었습니다.\n");

    if (interactive) {
        printf("종료하려면 Enter 또는 Ctrl+C를 누르세요.\n");

        while (!stop_requested) {
            struct pollfd input = {
                .fd = STDIN_FILENO,
                .events = POLLIN,
                .revents = 0,
            };

            const int result = poll(&input, 1, 250);

            if (result > 0) {
                if (input.revents & (POLLIN | POLLHUP | POLLERR)) {
                    char buffer[64];
                    const ssize_t bytes_read =
                        read(STDIN_FILENO, buffer, sizeof(buffer));

                    if (bytes_read < 0 && errno != EINTR) {
                        perror("입력 읽기 실패");
                    }

                    stop_requested = 1;
                }
            } else if (result < 0 && errno != EINTR) {
                perror("입력 대기 실패");
                stop_requested = 1;
            }
        }
    } else {
        printf("SIGINT 또는 SIGTERM을 받을 때까지 유지합니다.\n");

        while (!stop_requested) {
            pause();
        }
    }

    printf("TSC Generator 정지 명령 전송...\n");

    if (ioctl(fd, CDI_TSC_FSYNC, &off) < 0) {
        perror("TSC Generator OFF ioctl 실패");
        goto cleanup;
    }

    printf("FSYNC 펄스가 정상적으로 중지되었습니다.\n");
    exit_status = EXIT_SUCCESS;

cleanup:
    if (fd >= 0) {
        /*
         * 커널 드라이버의 release()에서도 발전기를 한 번 더 정지하고
         * 내부 FSYNC 상태로 복원한다.
         */
        if (close(fd) < 0) {
            perror("cdi_tsc close 실패");
            exit_status = EXIT_FAILURE;
        }
    }

    return exit_status;
}
