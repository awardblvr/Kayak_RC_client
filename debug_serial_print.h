//
// Created by Andrew Ward on 11/12/22.
//

#ifndef PRE_KAYAK_RC_CLIENT_DEBUG_SERIAL_PRINT_H
#define PRE_KAYAK_RC_CLIENT_DEBUG_SERIAL_PRINT_H

//#define STOP_ALL_SERIAL_IO

#if !defined(STOP_ALL_SERIAL_IO)
  #define debug_p_0()  Serial.print()
  #define debug_p_1(A)  Serial.print(A)
  #define debug_p_2(A,B) Serial.print(A,B)
  #define debug_pln_0()  Serial.println()
  #define debug_pln_1(A)  Serial.println(A)
  #define debug_pln_2(A,B) Serial.println(A,B)
#else
  #define debug_p_0()
  #define debug_p_1(A)
  #define debug_p_2(A,B)
  #define debug_pln_0()
  #define debug_pln_1(A)
  #define debug_pln_2(A,B)
#endif

// The interim macro that simply strips the excess and ends up with the required macro
#define debug_p_X(x,A,B,FUNC, ...)  FUNC

// The macro that the programmer uses
#define debug_p(...)  debug_p_X(,##__VA_ARGS__,\
                      debug_p_2(__VA_ARGS__),\
                      debug_p_1(__VA_ARGS__),\
                      debug_p_0(__VA_ARGS__)\
                )

// The interim macro that simply strips the excess and ends up with the required macro
#define debug_pln_X(x,A,B,FUNC, ...)  FUNC

// The macro that the programmer uses
#define debug_pln(...)  debug_pln_X(,##__VA_ARGS__,\
                      debug_pln_2(__VA_ARGS__),\
                      debug_pln_1(__VA_ARGS__),\
                      debug_pln_0(__VA_ARGS__)\
                )

                
#endif //PRE_KAYAK_RC_CLIENT_DEBUG_SERIAL_PRINT_H
