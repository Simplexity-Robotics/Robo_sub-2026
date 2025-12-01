// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice
#include "ht_messages/msg/detail/example__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `bar`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ht_messages__msg__Example__init(ht_messages__msg__Example * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ht_messages__msg__Example__fini(msg);
    return false;
  }
  // foo
  // bar
  if (!rosidl_runtime_c__float__Sequence__init(&msg->bar, 0)) {
    ht_messages__msg__Example__fini(msg);
    return false;
  }
  return true;
}

void
ht_messages__msg__Example__fini(ht_messages__msg__Example * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // foo
  // bar
  rosidl_runtime_c__float__Sequence__fini(&msg->bar);
}

bool
ht_messages__msg__Example__are_equal(const ht_messages__msg__Example * lhs, const ht_messages__msg__Example * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // foo
  if (lhs->foo != rhs->foo) {
    return false;
  }
  // bar
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->bar), &(rhs->bar)))
  {
    return false;
  }
  return true;
}

bool
ht_messages__msg__Example__copy(
  const ht_messages__msg__Example * input,
  ht_messages__msg__Example * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // foo
  output->foo = input->foo;
  // bar
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->bar), &(output->bar)))
  {
    return false;
  }
  return true;
}

ht_messages__msg__Example *
ht_messages__msg__Example__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ht_messages__msg__Example * msg = (ht_messages__msg__Example *)allocator.allocate(sizeof(ht_messages__msg__Example), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ht_messages__msg__Example));
  bool success = ht_messages__msg__Example__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ht_messages__msg__Example__destroy(ht_messages__msg__Example * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ht_messages__msg__Example__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ht_messages__msg__Example__Sequence__init(ht_messages__msg__Example__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ht_messages__msg__Example * data = NULL;

  if (size) {
    data = (ht_messages__msg__Example *)allocator.zero_allocate(size, sizeof(ht_messages__msg__Example), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ht_messages__msg__Example__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ht_messages__msg__Example__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ht_messages__msg__Example__Sequence__fini(ht_messages__msg__Example__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ht_messages__msg__Example__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ht_messages__msg__Example__Sequence *
ht_messages__msg__Example__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ht_messages__msg__Example__Sequence * array = (ht_messages__msg__Example__Sequence *)allocator.allocate(sizeof(ht_messages__msg__Example__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ht_messages__msg__Example__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ht_messages__msg__Example__Sequence__destroy(ht_messages__msg__Example__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ht_messages__msg__Example__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ht_messages__msg__Example__Sequence__are_equal(const ht_messages__msg__Example__Sequence * lhs, const ht_messages__msg__Example__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ht_messages__msg__Example__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ht_messages__msg__Example__Sequence__copy(
  const ht_messages__msg__Example__Sequence * input,
  ht_messages__msg__Example__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ht_messages__msg__Example);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ht_messages__msg__Example * data =
      (ht_messages__msg__Example *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ht_messages__msg__Example__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ht_messages__msg__Example__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ht_messages__msg__Example__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
