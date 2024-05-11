#include "XLRS.hpp"
#include "xlrs_mock_radio.hpp"
#include <unity.h>
#include <thread>
#include <future>
#include <mbedtls/aes.h>

using namespace radio::xlrs;

BlockingQueue<std::vector<uint8_t>> MockRadioDelegate::queueSideA;
BlockingQueue<std::vector<uint8_t>> MockRadioDelegate::queueSideB;

void setUp() {}
void tearDown() {}

const AES128Key test_pairing_key = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

void test_queue()
{
    BlockingQueue<int> queue1;
    queue1.push(1);
    queue1.push(2);
    TEST_ASSERT_EQUAL(queue1.pop(), 1);
    TEST_ASSERT_EQUAL(queue1.pop(), 2);
}

void test_mbedtls_works()
{
    TEST_ASSERT_EQUAL(0, mbedtls_aes_self_test(1));
}

void test_connect()
{
    auto t_req = std::async([]()
                            {
        MockRadioDelegate radio(false);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        TEST_ASSERT_TRUE(conn.requestConnect()); });
    auto t_res = std::async([]()
                            {
        MockRadioDelegate radio(true);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        TEST_ASSERT_TRUE(conn.respondConnect()); });

    t_res.wait();
    t_req.wait();
}

void test_l2()
{
    auto t_req = std::async([]()
                            {
        MockRadioDelegate radio(false);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.setSK(test_pairing_key);

        conn.sendL2Message((const uint8_t *)"Hello", strlen("Hello")); });
    auto t_res = std::async([]()
                            {
        MockRadioDelegate radio(true);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.setSK(test_pairing_key);

        uint8_t buf[256] = {0};
        TEST_ASSERT_EQUAL(strlen("Hello"), conn.receiveL2Message(buf, 256));
        TEST_ASSERT_EQUAL(0, strcmp((const char *)buf, "Hello")); });
    t_req.wait();
    t_res.wait();
}

void test_connect_and_then_send()
{
    auto t_req = std::async([]()
                            {
        MockRadioDelegate radio(false);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        TEST_ASSERT_TRUE(conn.requestConnect());
        conn.sendL2Message((const uint8_t *)"Hello", strlen("Hello")); });
    auto t_res = std::async([]()
                            {
        MockRadioDelegate radio(true);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        TEST_ASSERT_TRUE(conn.respondConnect());
        uint8_t buf[256] = {0};
        TEST_ASSERT_EQUAL(strlen("Hello"), conn.receiveL2Message(buf, 256));
        TEST_ASSERT_EQUAL(0, strcmp((const char *)buf, "Hello")); });

    t_res.wait();
    t_req.wait();
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_queue);
    RUN_TEST(test_mbedtls_works);
    RUN_TEST(test_connect);
    RUN_TEST(test_l2);
    RUN_TEST(test_connect_and_then_send);
    UNITY_END();

    return 0;
}