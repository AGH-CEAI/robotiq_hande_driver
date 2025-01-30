#include <cstdio>
#include <unistd.h>
#include "application.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world robotiq_hande_driver package\n");

  hande_driver::ApplicationLayer app_test = hande_driver::ApplicationLayer();
  app_test.activate();
  sleep(2);

  app_test.open();
  sleep(2);
  app_test.close();
  sleep(2);
  app_test.open();
  sleep(2);
  app_test.close();

  return 0;
}
