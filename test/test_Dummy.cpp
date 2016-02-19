#include <boost/test/unit_test.hpp>
#include <uwv_filters/Dummy.hpp>

using namespace uwv_filters;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    uwv_filters::DummyClass dummy;
    dummy.welcome();
}
