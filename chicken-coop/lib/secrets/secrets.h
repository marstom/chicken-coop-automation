/*
This helps to remember about create secrets_local.h file in  project with content like this:
*/
#pragma once

#if __has_include("secrets_local.h")
  #include "secrets_local.h"
#elif __has_include("secrets_prod.h")
  #include "secrets_prod.h"
#else
  #warning "Brak secrets_local.h - skopiuj secrets_local.example.h"
#endif
