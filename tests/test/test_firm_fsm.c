#include "unity.h"
#include "firm_fsm.h"
#include <string.h>

TaskCommand task_command_queue[MAX_TASK_COMMANDS];

void clear_queue() {
    memset(task_command_queue, 0, sizeof(task_command_queue));
}

void setUp(void) {
    clear_queue();
}

void tearDown(void) {}

void test_fsm_process_request_lifecycle(void) {
    // The only valid state we can start from is FIRM_BOOT
    TEST_ASSERT_EQUAL(FSMRES_INVALID, fsm_process_request(SYSREQ_FINISH_SETUP, task_command_queue));
    TEST_ASSERT_EQUAL(FSMRES_INVALID, fsm_process_request(SYSREQ_START_MOCK, task_command_queue));
    // Verify that the failure packet command was added to the queue
    TEST_ASSERT_EQUAL(TASKCMD_SYSTEM_PACKET_FAILURE, task_command_queue[0].command);
    clear_queue();
    TEST_ASSERT_EQUAL(FSMRES_INVALID, fsm_process_request(SYSREQ_CANCEL, task_command_queue));
    TEST_ASSERT_EQUAL(TASKCMD_SYSTEM_PACKET_FAILURE, task_command_queue[0].command);
    clear_queue();

    // Now send the valid SYSREQ_SETUP to move to FIRM_SETUP
    TEST_ASSERT_EQUAL(FSMRES_VALID, fsm_process_request(SYSREQ_SETUP, task_command_queue));

    // Verify the task commands populated for setup
    TEST_ASSERT_EQUAL(TASK_MODE_INDICATOR, task_command_queue[0].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SETUP, task_command_queue[0].command);
    TEST_ASSERT_EQUAL(TASK_DATA_FILTER, task_command_queue[1].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SETUP, task_command_queue[1].command);
    TEST_ASSERT_EQUAL(TASK_NULL, task_command_queue[2].target_task);
    clear_queue();

    // We are now in FIRM_SETUP. Attempts to start mock should fail
    TEST_ASSERT_EQUAL(FSMRES_INVALID, fsm_process_request(SYSREQ_START_MOCK, task_command_queue));
    clear_queue();
    // Send SYSREQ_FINISH_SETUP to move to FIRM_LIVE
    TEST_ASSERT_EQUAL(FSMRES_VALID, fsm_process_request(SYSREQ_FINISH_SETUP, task_command_queue));
    // Verify the task commands populated for live mode
    TEST_ASSERT_EQUAL(TASK_MODE_INDICATOR, task_command_queue[0].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_LIVE, task_command_queue[0].command);
    TEST_ASSERT_EQUAL(TASK_DATA_FILTER, task_command_queue[1].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_LIVE, task_command_queue[1].command);
    TEST_ASSERT_EQUAL(TASK_NULL, task_command_queue[2].target_task);
    clear_queue();

    // We are now in FIRM_LIVE. Attempts to cancel should fail as we are not mocking
    TEST_ASSERT_EQUAL(FSMRES_INVALID, fsm_process_request(SYSREQ_CANCEL, task_command_queue));
    TEST_ASSERT_EQUAL(TASKCMD_SYSTEM_PACKET_FAILURE, task_command_queue[0].command);
    clear_queue();
    // Send SYSREQ_START_MOCK to move to FIRM_MOCK
    TEST_ASSERT_EQUAL(FSMRES_VALID, fsm_process_request(SYSREQ_START_MOCK, task_command_queue));
    // Verify that all sensors and filters are set to mock, and packetizer reports success
    TEST_ASSERT_EQUAL(TASK_MODE_INDICATOR, task_command_queue[0].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_MOCK, task_command_queue[0].command);
    TEST_ASSERT_EQUAL(TASK_BMP581, task_command_queue[1].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_MOCK, task_command_queue[1].command);
    TEST_ASSERT_EQUAL(TASK_ICM45686, task_command_queue[2].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_MOCK, task_command_queue[2].command);
    TEST_ASSERT_EQUAL(TASK_MMC5983MA, task_command_queue[3].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_MOCK, task_command_queue[3].command);
    TEST_ASSERT_EQUAL(TASK_DATA_FILTER, task_command_queue[4].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_MOCK, task_command_queue[4].command);
    TEST_ASSERT_EQUAL(TASK_PACKETIZER, task_command_queue[5].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SYSTEM_PACKET_SUCCESS, task_command_queue[5].command);
    TEST_ASSERT_EQUAL(TASK_NULL, task_command_queue[6].target_task);
    clear_queue();

    // We are now in FIRM_MOCK. Send SYSREQ_CANCEL to return to FIRM_SETUP
    TEST_ASSERT_EQUAL(FSMRES_VALID, fsm_process_request(SYSREQ_CANCEL, task_command_queue));

    // Verify tasks are reset, sensors go to live, and packetizer reports success
    TEST_ASSERT_EQUAL(TASK_MODE_INDICATOR, task_command_queue[0].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SETUP, task_command_queue[0].command);
    TEST_ASSERT_EQUAL(TASK_BMP581, task_command_queue[1].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_LIVE, task_command_queue[1].command);
    TEST_ASSERT_EQUAL(TASK_ICM45686, task_command_queue[2].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_LIVE, task_command_queue[2].command);
    TEST_ASSERT_EQUAL(TASK_MMC5983MA, task_command_queue[3].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_LIVE, task_command_queue[3].command);
    TEST_ASSERT_EQUAL(TASK_DATA_FILTER, task_command_queue[4].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SETUP, task_command_queue[4].command);
    TEST_ASSERT_EQUAL(TASK_PACKETIZER, task_command_queue[5].target_task);
    TEST_ASSERT_EQUAL(TASKCMD_SYSTEM_PACKET_SUCCESS, task_command_queue[5].command);
    TEST_ASSERT_EQUAL(TASK_NULL, task_command_queue[6].target_task);
}