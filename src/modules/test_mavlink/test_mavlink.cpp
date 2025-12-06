#include "test_mavlink.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>



#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int test_mavlink::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int test_mavlink::custom_command(int argc, char *argv[])
{
    /*
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "do-something")) {
        get_instance()->do_something();
        return 0;
    }
     */

    return print_usage("unknown command");
}


int test_mavlink::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0)
    {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

test_mavlink *test_mavlink::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF)
    {
        switch (ch)
        {
            case 'p':
                example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag)
    {
        return nullptr;
    }

    test_mavlink *instance = new test_mavlink(example_param, example_flag);

    if (instance == nullptr)
    {
        PX4_ERR("alloc failed");
    }

    return instance;
}

test_mavlink::test_mavlink(int example_param, bool example_flag)
    : ModuleParams(nullptr)
{
	_loop_perf = perf_alloc(PC_ELAPSED, "my_module_loop");

}

void test_mavlink::run()
{



    // initialize parameters
    parameters_update(true);

   while (!should_exit())
    {
	perf_begin(_loop_perf);
	px4_usleep(100000); // 每 10ms 循环一次

        if(test_mavlink_rx_sub.updated())
        {

             test_mavlink_rx_sub.copy(&_test_mavlink_rx);
             PX4_INFO("_test_mavlink_rx.test1=%d\n",_test_mavlink_rx.test1);
             PX4_INFO("_test_mavlink_rx.test2=%d\n",_test_mavlink_rx.test2);
             PX4_INFO("_test_mavlink_rx.test3=%f\n",(double)_test_mavlink_rx.test3);
            _test_mavlink_tx.test1=_test_mavlink_rx.test1+1;
            _test_mavlink_tx.test2=_test_mavlink_rx.test2+1;
            _test_mavlink_tx.test3=_test_mavlink_rx.test3+1;
            test_mavlink_tx_pub.publish(_test_mavlink_tx);
	}
	//PX4_INFO("we test here!!!!!!!!!!!");
	// PX4_INFO("TEST_MAVLINK _leader_info_sub.updated() is %d", _leader_info_sub.updated());
    // _leader_info_sub.copy(&_leader_info);
    // PX4_INFO("_leader_info.vz is %f",_leader_info.vz);
    // PX4_INFO("_leader_info.yaw is %f",_leader_info.yaw);
    // PX4_INFO("_leader_info.rel_alt is %f",_leader_info.alt);
    //  static int iteration_count = 0;
    //             iteration_count++;
    // PX4_INFO("iteration_count is %d", iteration_count);
	 
     if(_leader_info_sub.updated())
        {

            _leader_info_sub.copy(&_leader_info);
            aa.lat=_leader_info.lat;
            aa.lon=_leader_info.lon;
            aa.vx=_leader_info.vx;
           // PX4_INFO("_leader_info.vz is %f",_leader_info.vz);
	       // PX4_INFO("_leader_info.yaw is %f",_leader_info.yaw);
	       // PX4_INFO("_leader_info.rel_alt is %f",_leader_info.alt);
	}

      //  PX4_INFO("New line");
      //  PX4_INFO("_test_mavlink_rx.test1=%d\n",_test_mavlink_rx.test1);
      //  PX4_INFO("_test_mavlink_rx.test2=%d\n",_test_mavlink_rx.test2);
      //  PX4_INFO("_test_mavlink_rx.test3=%f\n",(double)_test_mavlink_rx.test3);
         // 控制日志输出频率



     	parameters_update();
	perf_end(_loop_perf);

    }

}

void test_mavlink::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force)
    {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        updateParams();
    }
}

int test_mavlink::print_usage(const char *reason)
{
    if (reason)
    {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int test_mavlink_main(int argc, char *argv[])
{
	return test_mavlink::main(argc, argv);
}
