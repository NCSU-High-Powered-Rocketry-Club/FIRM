#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "data_preprocess.h"
#include "mocking_handler.h"

typedef struct {
  MockPacketID id;
  SensorPacket packet;
} MockSensorPacket;

typedef struct {
  MockSensorPacket *storage;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
} MockRingBuffer;

void mock_ring_init(MockRingBuffer *rb, MockSensorPacket *storage, size_t capacity);
void mock_ring_reset(MockRingBuffer *rb);

MockSensorPacket *mock_ring_reserve_packet(MockRingBuffer *rb);

void mock_ring_commit_reserved(MockRingBuffer *rb);

bool mock_ring_pop(MockRingBuffer *rb, MockSensorPacket *out);

bool mock_ring_peek(const MockRingBuffer *rb, MockSensorPacket *out);

size_t mock_ring_get_length(const MockRingBuffer *rb);
