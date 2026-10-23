#include <stddef.h>
#include <stdlib.h>

void * microros_allocate(size_t size, void * state)
{
  (void) state;
  return malloc(size);
}

void microros_deallocate(void * pointer, void * state)
{
  (void) state;
  free(pointer);
}

void * microros_reallocate(void * pointer, size_t size, void * state)
{
  (void) state;
  return realloc(pointer, size);
}

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  (void) state;
  return calloc(number_of_elements, size_of_element);
}
