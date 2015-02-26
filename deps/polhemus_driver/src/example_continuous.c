#include <stdio.h>
#include <unistd.h>

#include "polhemus_driver/polhemus_driver.h"

void
pose_cb(double x, double y, double z,
        double roll, double pitch, double yaw)
{
  printf("%lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
}

int
main(void)
{
  polhemus_conn_t* conn;

  if(!(conn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                   LIBERTY_HS_PRODUCT_ID,
                                   LIBERTY_HS_WRITE_ENDPOINT,
                                   LIBERTY_HS_READ_ENDPOINT)))
  {
    fprintf(stderr, "Failed to connect\n");
    return -1;
  }

  if(polhemus_init_comm(conn, 10))
  {
    fprintf(stderr, "Failed to initialize comms\n");
    return -1;
  }

  printf("Going into continuous mode\n");
  polhemus_start_continuous_mode(conn, pose_cb);
  usleep(5000000);
  printf("Stopping continuous mode\n");
  polhemus_stop_continuous_mode(conn);

  polhemus_disconnect_usb(conn);
  return 0;
}
