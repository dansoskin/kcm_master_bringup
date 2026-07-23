/*
 * functions.h
 *
 *  Application setup() / loop() entry points called from main().
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

/* Called once from main() after the peripherals are initialised. */
void setup(void);

/* Called repeatedly from the main() while(1) loop. */
void loop(void);

#endif /* INC_FUNCTIONS_H_ */
