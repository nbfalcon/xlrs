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

void test_connect()
{
    TEST_ASSERT_EQUAL(mbedtls_aes_self_test(1), 0);

    auto t_req = std::async([]()
                            {
        MockRadioDelegate radio(false);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        return conn.requestConnect(); });
    auto t_res = std::async([]()
                            {
        MockRadioDelegate radio(true);
        XLRSConnection conn(static_cast<RadioDelegate *>(&radio));
        conn.init(test_pairing_key);

        return conn.respondConnect(); });

    TEST_ASSERT_TRUE(t_req.get());
    TEST_ASSERT_TRUE(t_res.get());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_queue);
    RUN_TEST(test_connect);
    UNITY_END();

    return 0;
}