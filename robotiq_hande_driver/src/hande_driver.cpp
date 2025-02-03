#include <cstdio>
#include <unistd.h>

#include "application.hpp"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world robotiq_hande_driver package4\n");
  hande_driver::ApplicationLayer app_test;

  printf("Activation robotiq_hande_driver\n");
  app_test.Activate();
  sleep(4);

  printf("Open-Close loop x2\n");

  printf("Open\n");
  app_test.Open();
  sleep(2);
  printf("Close\n");
  app_test.Close();
  sleep(2);

  printf("Open\n");
  app_test.Open();
  sleep(2);
  printf("Close\n");
  app_test.Close();

  printf("Finished\n");

  return 0;
}
