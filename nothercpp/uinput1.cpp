// https://stackoverflow.com/questions/76985642/is-there-a-way-to-set-absolute-cursor-position-for-uinput-virtual-device-waylan

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/uinput.h>
#include <sys/ioctl.h>

static void fatal(const char *msg)
{
    fprintf(stderr, "fatal: ");

    if (errno)
        perror(msg);
    else
        fprintf(stderr, "%s\n", msg);

    exit(EXIT_FAILURE);
}

static void setup_abs(int fd, int type, int min, int max, int res)
{
    struct uinput_abs_setup abs = {
        .code = type,
        .absinfo = {
            .minimum = min,
            .maximum = max,
            .resolution = res
        }
    };

    if (-1 == ioctl(fd, UI_ABS_SETUP, &abs))
        fatal("ioctl UI_ABS_SETUP");
}

static void init(int fd, int width, int height, int dpi)
{
    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_SYN))
        fatal("ioctl UI_SET_EVBIT EV_SYN");

    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_KEY))
        fatal("ioctl UI_SET_EVBIT EV_KEY");
    if (-1 == ioctl(fd, UI_SET_KEYBIT, BTN_LEFT))
        fatal("ioctl UI_SET_KEYBIT BTN_LEFT");

    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_ABS))
        fatal("ioctl UI_SET_EVBIT EV_ABS");
    /* the ioctl UI_ABS_SETUP enables these automatically, when appropriate:
        ioctl(fd, UI_SET_ABSBIT, ABS_X);
        ioctl(fd, UI_SET_ABSBIT, ABS_Y);
    */

    struct uinput_setup device = {
        .id = {
            .bustype = BUS_USB
        },
        .name = "Emulated Absolute Positioning Device"
    };

    if (-1 == ioctl(fd, UI_DEV_SETUP, &device))
        fatal("ioctl UI_DEV_SETUP");

    setup_abs(fd, ABS_X, 0, width, dpi);
    setup_abs(fd, ABS_Y, 0, height, dpi);

    if (-1 == ioctl(fd, UI_DEV_CREATE))
        fatal("ioctl UI_DEV_CREATE");

    /* give time for device creation */
    sleep(1);
}

static void emit(int fd, int type, int code, int value)
{
    struct input_event ie = {
        .type = type,
        .code = code,
        .value = value
    };

    write(fd, &ie, sizeof ie);
}

int main(int argc, char **argv)
{
    /* These values are very device specific */
    int w = argc > 1 ? atoi(argv[1]) : 1920;
    int h = argc > 2 ? atoi(argv[2]) : 1080;
    int d = argc > 3 ? atoi(argv[3]) : 96;

    if (w < 1 || h < 1 || d < 1)
        fatal("Bad initial value(s).");

    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

    if (-1 == fd)
        fatal("open");

    printf("Initializing device screen map as %dx%d @ %ddpi\n", w, h, d);

    init(fd, w, h, d);

    while (1) {
        printf("Enter x & y: ");
        fflush(stdout);

        char input[128];
        if (!fgets(input, sizeof input, stdin) || 0 == strncmp(".exit", input, 5))
            break;

        int x, y;
        if (2 != sscanf(input, "%d%d", &x, &y) || x < 0 || y < 0) {
            fprintf(stderr, "Invalid input.\n");
            continue;
        }

        printf("Moving cursor to %d,%d\n", x, y);

        /* input is zero-based, but event positions are one-based */
        emit(fd, EV_ABS, ABS_X, 1 + x);
        emit(fd, EV_ABS, ABS_Y, 1 + y);
        emit(fd, EV_SYN, SYN_REPORT, 0);
    }

    puts("Cleaning up...");

    /* give time for events to finish */
    sleep(1);

    if (-1 == ioctl(fd, UI_DEV_DESTROY))
        fatal("ioctl UI_DEV_DESTROY");

    close(fd);
    puts("Goodbye.");
}