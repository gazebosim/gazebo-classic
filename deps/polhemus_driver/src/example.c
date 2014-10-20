#include <stdio.h>
#include <unistd.h>

#include "polhemus_driver/polhemus_driver.h"

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

  double x, y, z, roll, pitch, yaw;
  for(;;)
  {
    polhemus_get_pose(conn, &x, &y, &z, &roll, &pitch, &yaw, 10);
    printf("%lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
  }

  polhemus_disconnect_usb(conn);
  return 0;
}
