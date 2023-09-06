# 简介

适用于**V9.97**

项目的开源地址在[这里](https://github.com/easylogging/easyloggingpp)

该Log库只需包含一个头文件一个源文件即可，轻量且易于融入项目中。**需要c++11**

# 简单使用

```cpp
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[]) {
   LOG(INFO) << "My first info log using default logger";
   return 0;
}
```

只需要包含头文件并使用宏`INITIALIZE_EASYLOGGING`初始化，使用`LOG(INFO)`即可。

`INITIALIZE_EASYLOGGING`的作用是初始化一些变量，全局只能使用一次。因此可以考虑在`main`函数的文件中去声明。

# 通过传递启动参数指定功能

```cpp
int main(int argc, char* argv[]) {
   START_EASYLOGGINGPP(argc, argv);
   ...
}
```

功能待补充。。。

# 等级标签

| **等级**  | **描述**                       |
|:------- |:---------------------------- |
| Global  | 表示所有级别的通用级别。在为所有级别设置全局配置时很有用 |
| Trace   | 可用于回溯某些事件的信息-----比Debug日志更有用 |
| Debug   | 普通的Debug级别                   |
| Fatal   | 导致应用程序终止的严重的事件               |
| Error   | 是错误，但是程序会继续运行                |
| Warning | 是小点的错误，程序会继续运行               |
| Info    | 当前应用程序走到哪了的一些信息              |
| Verbose | 随详细日志记录级别的不同而变化的信息           |
| Unknown | 仅用于分等级日志记录，用于关闭日志记录          |

# 配置

可以通过三种方式进行配置：

1. 配置文件

2. el::Configurations 类

3. 内联配置

## 配置文件

配置文件可以通过Configurations类在运行时加载的文件来完成。此文件具有以下格式：

```xml
* LEVEL:
  CONFIGURATION NAME  = "VALUE" ## Comment
  ANOTHER CONFIG NAME = "VALUE"
```

级别名称以`*`号开头以`:`结尾，通常建议以`Global`级别作为开头进行配置，以便对为配置的级别，在代码中可以使用`Global`的配置进行加载。

下表为相应的`CONFIGURATION NAME`：

| Configuration Name     | **类型** | **描述**                                         |
| ---------------------- | ------ | ---------------------------------------------- |
| `Enabled`              | bool   | 确定是否启用记录器的相应级别。您可以使用el::Level::Global禁用所有日志    |
| `To_File`              | bool   | 是否将相应的日志写到日志文件                                 |
| `To_Standard_Output`   | bool   | 是否输出到终端                                        |
| `Format`               | char*  | 定义日志输出格式                                       |
| `Filename`             | char*  | 不同级别和不同记录器写入文件的文件名，需要完整路径                      |
| `Subsecond_Precision`  | uint   | 指定亚秒精度（以前称为“毫秒宽度”）。值可以在（1-6）范围内                |
| `Performance_Tracking` | bool   | 确定是否启用性能跟踪。这并不取决于记录器或级别。除非指定，否则性能跟踪始终使用“性能”记录器 |
| `Max_Log_File_Size`    | size_t | 如果相应级别的日志文件大小>=指定大小，则日志文件将被截断。                 |
| `Log_Flush_Threshold`  | size_t | 指定在刷新挂起的日志数据之前要保留的日志条目数                        |

### 例子：

```xml
* GLOBAL:
   FORMAT               =  "%datetime %msg"
   FILENAME             =  "/tmp/logs/my.log"
   ENABLED              =  true
   TO_FILE              =  true
   TO_STANDARD_OUTPUT   =  true
   SUBSECOND_PRECISION  =  6
   PERFORMANCE_TRACKING =  true
   MAX_LOG_FILE_SIZE    =  2097152 ## 2MB - Comment starts with two hashes (##)
   LOG_FLUSH_THRESHOLD  =  100 ## Flush after every 100 logs
* DEBUG:
   FORMAT               = "%datetime{%d/%M} %func %msg"
```

**注**：在`##`后边的内容为注释，在注释中不要使用双引号

例子解释：

首先通过`Global`进行全局配置，再单独对`DEBUG`级别的`Log`记录输出`时间、日期、年份`默认的`%daratime`是包括`年月日时分秒`，再对`DEBUG`进行单独配置，将输出格式中只显示`日月 函数 消息`

### 加载

可以使用如下方式对配置文件进行加载：

```cpp
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, const char** argv) {
    // 从文件加载配置文件
    el::Configurations conf("/path/to/my-conf.conf");
    // 为单独一个级别的Logger配置
    el::Loggers::reconfigureLogger("default", conf);
    // 为全部的Logger配置
    el::Loggers::reconfigureAllLoggers(conf);
    // Now all the loggers will use configuration from file
    // 也可直接将加载的配置文件设置为默认的配置
    el::Loggers::setDefaultConfigurations( conf );
    // 这样后注册的所有Logger，都是默认配置，而无需重复配置
}
```

## el::Configuration类

对加载的进程序的文件进行二次修改：

```cpp
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, const char** argv) {
   el::Configurations defaultConf;
   defaultConf.setToDefault();
    // 通过std::string进行重新配置
   defaultConf.set(el::Level::Info,
            el::ConfigurationType::Format, "%datetime %level %msg");
    // default logger uses default configurations
    el::Loggers::reconfigureLogger("default", defaultConf);
    LOG(INFO) << "Log using default file";
    // To set GLOBAL configurations you may use
   defaultConf.setGlobally(
            el::ConfigurationType::Format, "%date %msg");
   el::Loggers::reconfigureLogger("default", defaultConf);
    return 0;
}
```

直接在代码中进行配置，需要好多换行符，不推荐，不直观

```cpp
el::Configurations c;
c.setToDefault();
c.parseFromText("*GLOBAL:\n FORMAT = %level %msg");
```

# 日志输出格式

可以通过配置文件的`FORMAT`标签进行输出格式的配置

| **标签**         | 描述                                                                 |
| -------------- | ------------------------------------------------------------------ |
| `%logger`      | Logger ID                                                          |
| `%thread`      | Thread ID -- 如果可用，则使用std:：thread，否则在windows上使用GetCurrentThreadId（） |
| `%thread_name` | 使用`Helpers:：setThreadName`设置当前线程的名称                                |
| `%level`       | 日志级别                                                               |
| `%levshort`    | 日志级别简写                                                             |
| `%vlevel`      | 日志级别详细                                                             |
| `%datetime`    | 时间                                                                 |
| `%user`        | 当前的user,/home/user,指的是这个user                                       |
| `%host`        | 当前ip                                                               |
| `%file*`       | 源文件名称，需要支持`__FILE__`宏，会打印全完整路径+文件名                                 |
| `%fbase*`      | 源文件名称，不包含完整路径                                                      |
| `%line*`       | 源文件行号，需支持`__line__`宏                                               |
| `%func*`       | 运行的函数                                                              |
| `%loc*`        | 源文件:行号，其中源文件为完整路径+文件名                                              |
| `%msg`         | 要打印的日志信息                                                           |
| `%`            | 转义符，如：%%level 输出的内容是 %level                                        |

要使用`%host`需要做如下设置：

```cpp
const char* getIp(const el::LogMessage*) {
    return "192.168.1.1";
}

int main(void) {
    el::Helpers::installCustomFormatSpecifier(el::CustomFormatSpecifier("%ip_addr", getIp));
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%datetime %level %ip_addr : %msg");
    LOG(INFO) << "This is request from client";
    return 0;
}
```

## 日期格式

| 标签   | 描述                                                                                                      |
| ---- | ------------------------------------------------------------------------------------------------------- |
| `%d` | 日期（0填充）                                                                                                 |
| `%a` | 周，缩写 (Mon, Tue, Wed, Thu, Fri, Sat, Sun)                                                                |
| `%A` | 周，全称 (Monday, Tuesday, Wednesday, Thursday, Friday, Saturday, Sunday)                                   |
| `%M` | 月（0填充）                                                                                                  |
| `%b` | 月，缩写 (Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec)                                       |
| `%B` | 月，全称 (January, February, March, April, May, June, July, August, September, October, November, December) |
| `%y` | 年，只是后两位 (13, 14 etc)                                                                                    |
| `%Y` | 年，四位 (2013, 2014 etc)                                                                                   |
| `%h` | 小时，12小时制                                                                                                |
| `%H` | 小时，24小时制                                                                                                |
| `%m` | 分钟（0填充）                                                                                                 |
| `%s` | 秒（0填充）                                                                                                  |
| `%g` | Subsecond part (precision is configured by ConfigurationType::SubsecondPrecision)                       |
| `%F` | AM/PM                                                                                                   |
| `%`  | 转义符                                                                                                     |

**请注意，日期/时间最多限制为30个字符。**

# 日志标签

以下是支持的标志：

| 标签                                                     | 描述                                                                                                                                                                                                                                                                                  |
| ------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `NewLineForContainer (1)`                              | 确保每个容器日志条目都有新行                                                                                                                                                                                                                                                                      |
| `AllowVerboseIfModuleNotSpecified (2)`                 | 确保如果使用了-vmodule并且没有指定模块，则允许通过该模块进行详细日志记录。假设param was-vmodule=main*=3，并且正在从名为something.cpp的文件中写入详细日志，则如果启用了此标志，则将写入日志，否则将不允许写入。注意：这样做违背了-vmodule的目的                                                                                                                                  |
| `LogDetailedCrashReason (4)`                           | 在默认情况下处理崩溃时，还会记录详细的崩溃原因（默认情况下已禁用） ([issue #90](https://github.com/abumq/easyloggingpp/issues/90))                                                                                                                                                                                   |
| `DisableApplicationAbortOnFatalLog (8)`                | 允许在使用FATAL级别登录时禁用应用程序中止。请注意，这不适用于默认的崩溃处理程序，因为在处理崩溃信号后应用程序应该中止（默认情况下不添加） ([issue #119](https://github.com/abumq/easyloggingpp/issues/119))                                                                                                                                           |
| `ImmediateFlush (16)`                                  | 使用每个日志条目刷新日志（性能敏感）-默认情况下禁用                                                                                                                                                                                                                                                          |
| `StrictLogFileSizeCheck (32)`                          | 确保与每个日志一起检查日志文件大小                                                                                                                                                                                                                                                                   |
| `ColoredTerminalOutput (64)`                           | 如果终端支持，终端输出将是彩色。                                                                                                                                                                                                                                                                    |
| `MultiLoggerSupport (128)`                             | 支持使用多个记录器记录单个消息。（例如，`CLOG（INFO，“default”，“network”）<<This will be logged using default and network loggers;`）                                                                                                                                                                       |
| `DisablePerformanceTrackingCheckpointComparison (256)` | 禁用检查点比较                                                                                                                                                                                                                                                                             |
| `DisableVModules (512)`                                | 禁用vmodules的使用                                                                                                                                                                                                                                                                       |
| `DisableVModulesExtensions (1024)`                     | 禁用`vmodules`扩展。这意味着，如果您有一个`vmodule-vmodule=main*=4`，它将覆盖以`main`开头的所有内容，其中，如果您没有定义它，则将覆盖以`main`开头并以以下扩展名之一结尾的任何文件`.h.c.cpp.cc.cxx.-inl-.h.xxx.hpp`。请注意以下`vmodule`不正确`-vmodule=main=4`未定义此宏，因为这将检查`main..c`、 注意双点。如果要使其有效，请查看上面的日志记录标志：`AllowVerboseIfModuleNotSpecified``?`和``通配符受支持 |
| `HierarchicalLogging (2048)`                           | 启用分层日志记录。这不适用于详细日志记录。                                                                                                                                                                                                                                                               |
| `CreateLoggerAutomatically (4096)`                     | 当禁止时自动创建log记录器                                                                                                                                                                                                                                                                      |
| `AutoSpacing (8192)`                                   | 自动添加空格E.g, `LOG(INFO) << "DODGE" << "THIS!";` 输出`"DODGE THIS!"`                                                                                                                                                                                                                     |
| `FixedTimeFormat (16384)`                              | 仅适用于性能跟踪-这样可以防止格式化时间。例如，1001毫秒将按原样记录，而不是将其格式化为1.01秒                                                                                                                                                                                                                                 |
| `IgnoreSigInt (32768)`                                 | 应用程序崩溃时忽略中断信号                                                                                                                                                                                                                                                                       |

以上标签可以通过`el::Loggers::addFlag`进行设置，也可以通过`el::Loggers::removeFlag`解除设置，可以通过`el::Loggers::hasFlag`检查标签是否设置，所有这些函数都采用强类型枚举`el::LoggingFlag`

> 可以使用`--logging flags`命令行`arg`设置这些标志。需要通过定义宏`ELPP_LOGGING_FLAGS_FROM_ARG`来启用此功能（需要确保使用`START_EASYLOGGINGPP(argc，argv)`来接收配置参数）。

> 也可以使用`ELPP_default_LOGGING_flags`设置默认（初始）标志，并设置初始标志的数值

必须设置标记`LoggingFlag::StrictLogFileSizeCheck`否则,配置文件中`MAX_LOG_FILE_SIZE = 1048576`不生效 

# 应用程序参数

下表将解释您可以用来定义某些行为的所有命令行参数；您需要在`main(int，char**)`函数中使用`START_EASYLOGGINGPP(argc，argv)`来初始化应用程序参数。

| 参数                        | 描述                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-v`                      | 激活最大详细log记录                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--v=2`                   | 激活详细程度2以下的 (可取范围: 0-9)                                                                                                                                                                                                                                                                                                                                                                                             |
| `--verbose`               | 同`-v`                                                                                                                                                                                                                                                                                                                                                                                                              |
| `-vmodule=MODULE_NAME`    | Activates verbosity for files starting with main to level 1, the rest of the files depend on logging flag `AllowVerboseIfModuleNotSpecified` Please see Logging Flags section above. Two modules can be separated by comma. Please note vmodules are last in order of precedence of checking arguments for verbose logging, e.g, if we have -v in application arguments before vmodules, vmodules will be ignored. |
| `--logging-flags=3`       | Sets logging flag. In example `i.e, 3`, it sets logging flag to `NewLineForContainer` and `AllowVerboseIfModuleNotSpecified`. See logging flags section above for further details and values. See macros section to disable this function.                                                                                                                                                                         |
| `--default-log-file=FILE` | 设置现有和未来记录器的默认日志文件。您可能需要考虑定义`ELPP_NO_DEFAULT_LOG_FILE`，以防止在预处理期间创建默认的空日志文件。请参阅宏部分禁用此功能。                                                                                                                                                                                                                                                                                                                             |

# 配置宏

一些日志记录选项可以由宏设置，这是一个深思熟虑的决定，例如，如果我们定义了`ELPP_THREAD_SAFE`，则所有线程安全功能都会被启用否则禁用（确保线程安全性与此相关）。为了便于记忆并防止可能的冲突，所有宏都以`ELPP_`开头

**注意：所有宏都可以通过以下方式之一定义：**

1. 使用编译器的`-D`选项定义宏，例如在`g++`的情况下，您将执行`g++ source.cpp…-DELPP_SYSLOG-DELPP_THREAD_SAFE`（推荐方式）

2. 在“`easylogging++.h`”中定义宏（在其他文件中定义宏不起作用）

| 宏名称                                     | 描述                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ELPP_DEBUG_ASSERT_FAILURE`             | 第一次断言失败时中止应用程序。此断言是由于无效的输入，例如无效的配置文件等                                                                                                                                                                                                                                                                                                                                 |
| `ELPP_UNICODE`                          | 在日志记录时启用Unicode支持。需要`START_ASYLOGGINGPP`                                                                                                                                                                                                                                                                                                                              |
| `ELPP_THREAD_SAFE`                      | 为linux启用线程安全-确保`-lpthread`链接。                                                                                                                                                                                                                                                                                                                                         |
| `ELPP_FORCE_USE_STD_THREAD`             | 强制使用C++标准库进行线程处理（仅在使用`ELPP_THREAD_SAFE`时有用                                                                                                                                                                                                                                                                                                                            |
| `ELPP_FEATURE_CRASH_LOG`                | 仅适用于GCC。在应用程序崩溃时启用堆栈跟踪                                                                                                                                                                                                                                                                                                                                                |
| `ELPP_DISABLE_DEFAULT_CRASH_HANDLING`   | 禁用默认故障处理。您可以使用`el::Helpers::setCrashHandler`来使用自己的处理程序。                                                                                                                                                                                                                                                                                                               |
| `ELPP_DISABLE_LOGS`                     | 禁用所有日志-（预处理）                                                                                                                                                                                                                                                                                                                                                          |
| `ELPP_DISABLE_DEBUG_LOGS`               | 禁用DEBUG日志-（预处理）                                                                                                                                                                                                                                                                                                                                                       |
| `ELPP_DISABLE_INFO_LOGS`                | 禁用INFO日志-（预处理）                                                                                                                                                                                                                                                                                                                                                        |
| `ELPP_DISABLE_WARNING_LOGS`             | 禁用WARING日志-（预处理）                                                                                                                                                                                                                                                                                                                                                      |
| `ELPP_DISABLE_ERROR_LOGS`               | 禁用ERROR日志-（预处理）                                                                                                                                                                                                                                                                                                                                                       |
| `ELPP_DISABLE_FATAL_LOGS`               | 禁用FATAL日志-（预处理）                                                                                                                                                                                                                                                                                                                                                       |
| `ELPP_DISABLE_VERBOSE_LOGS`             | 禁用VERBOSE日志-（预处理）                                                                                                                                                                                                                                                                                                                                                     |
| `ELPP_DISABLE_TRACE_LOGS`               | 禁用TRACE日志-（预处理）                                                                                                                                                                                                                                                                                                                                                       |
| `ELPP_FORCE_ENV_VAR_FROM_BASH`          | If environment variable could not be found, force using alternative bash command to find value, e.g, `whoami` for username. (DO NOT USE THIS MACRO WITH `LD_PRELOAD` FOR LIBRARIES THAT ARE ALREADY USING Easylogging++ OR YOU WILL END UP IN STACK OVERFLOW FOR PROCESSES (`popen`) (see [issue #87](https://github.com/abumq/easyloggingpp/issues/87) for details)) |
| `ELPP_DEFAULT_LOG_FILE`                 | 要创建初始文件的完整文件名。您需要用引号嵌入这个宏的值， e.g, `-DELPP_DEFAULT_LOG_FILE='"logs/el.gtest.log"'` 注意单引号中的双引号，双引号是`const char*`的值，单引号指定宏的值                                                                                                                                                                                                                                             |
| `ELPP_NO_LOG_TO_FILE`                   | 最初禁用对文件的日志记录                                                                                                                                                                                                                                                                                                                                                          |
| `ELPP_NO_DEFAULT_LOG_FILE`              | 若不想用默认的日志文件初始化库，请定义这个宏。这将记录到`unix`和`windows`的`null`设备。在其他平台中，您可能会遇到错误，需要使用`ELPP_DEFAULT_LOG_FILE`。                                                                                                                                                                                                                                                                    |
| `ELPP_FRESH_LOG_FILE`                   | 无论是否创建了log文件，都不续写                                                                                                                                                                                                                                                                                                                                                     |
| `ELPP_DEBUG_ERRORS`                     | 打开easylogging++库的log                                                                                                                                                                                                                                                                                                                                                  |
| `ELPP_DISABLE_CUSTOM_FORMAT_SPECIFIERS` | 强制禁用自定义格式说明符                                                                                                                                                                                                                                                                                                                                                          |
| `ELPP_DISABLE_LOGGING_FLAGS_FROM_ARG`   | 强制禁用使用命令行参数设置日志记录标志的功能                                                                                                                                                                                                                                                                                                                                                |
| `ELPP_DISABLE_LOG_FILE_FROM_ARG`        | 强制禁用从命令行参数设置默认日志文件的功能                                                                                                                                                                                                                                                                                                                                                 |
| `ELPP_WINSOCK2`                         | 在windows系统上，当定义`WIN32_EAN_AND_MEAN`时，强制使用`winsock2.h`而不是`winsock.h`                                                                                                                                                                                                                                                                                                   |
| `ELPP_CUSTOM_COUT` (advanced)           | Resolves to a value e.g, `#define ELPP_CUSTOM_COUT qDebug()` or `#define ELPP_CUSTOM_COUT std::cerr`. This will use the value for standard output (instead of using `std::cout`                                                                                                                                                                                       |
| `ELPP_CUSTOM_COUT_LINE` (advanced)      | Used with `ELPP_CUSTOM_COUT` to define how to write a log line with custom cout. e.g, `#define ELPP_CUSTOM_COUT_LINE(msg) QString::fromStdString(msg).trimmed()`                                                                                                                                                                                                      |
| `ELPP_NO_CHECK_MACROS`                  | 不要定义*CHECK*宏                                                                                                                                                                                                                                                                                                                                                          |
| `ELPP_NO_DEBUG_MACROS`                  | 不要定义*DEBUG*宏                                                                                                                                                                                                                                                                                                                                                          |
| `ELPP_UTC_DATETIME`                     | 使用UTC时间而不是当地时间                                                                                                                                                                                                                                                                                                                                                        |
| `ELPP_NO_GLOBAL_LOCK`                   | Do not lock the whole storage on dispatch. This should be used with care. See [issue #580](https://github.com/abumq/easyloggingpp/issues/580)                                                                                                                                                                                                                         |

# 读取配置

通过代码读取某一个`logger`的配置，可以通过：

```cpp
el::Logger* l = el::Loggers::getLogger("default");
bool enabled = l->typedConfigurations()->enabled(el::Level::Info);
// Or to read log format/pattern
std::string format =
        l->typedConfigurations()->logFormat(el::Level::Info).format();
```

# 记录日志

## 普通日志记录

可以使用这两个宏记录日志:

- `LOG(LEVEL)`
- `CLOG(LEVEL, logger ID)`

`LOG`使用的默认的记录器，`CLOG`可以选择使用用户自定义的记录器，可以让不同的`Logger`有不同的配置。

通常在使用CLOG前应该先对Logger进行注册

例子如下：

```cpp
LOG(INFO) << "This is info log";
el::Loggers::getLogger( "performance" );
CLOG(ERROR, "performance") << "This is info log using performance logger";
```

还可以通过修改`LOG`宏的默认`Logger`，以达到自定义效果，需要在`.cc`文件中定义。

```cpp
#ifndef ELPP_DEFAULT_LOGGER
#   define ELPP_DEFAULT_LOGGER "update_manager"
#endif
#ifndef ELPP_DEFAULT_PERFORMANCE_LOGGER
#   define ELPP_DEFAULT_PERFORMANCE_LOGGER ELPP_DEFAULT_LOGGER
#endif
#include "easylogging++.h"
UpdateManager::UpdateManager {
    _TRACE; // Logs using LOG(TRACE) provided logger is already registered - i.e, update_manager
    LOG(INFO) << "This will log using update_manager logger as well";
}
```

```cpp
#include "easylogging++.h"
UpdateManager::UpdateManager {
    _TRACE; // Logs using LOG(TRACE) using default logger because no `ELPP_DEFAULT_LOGGER` is defined unless you have it in makefile
}
```

## 条件日志记录

1. 每多少次记录1次
   
   例如每10次、第100次或第2次，就会写入日志。帮助程序宏以`_EVERY_N`结尾；
- `LOG_EVERY_N(n, LEVEL)`
- `CLOG_EVERY_N(n, LEVEL, logger ID)`

```cpp
for ( int i = 0; i < 10; i++ )
{
    CLOG_EVERY_N( 6, ERROR, "map" ) << "This is a CLOG_IF ERROR log for map logger = " << i;
}
// 输出：This is a CLOG_EVERY_N ERROR log for map logger = 5
```

2. 多少次之后开始记录
- `LOG_AFTER_N(n, LEVEL)`
- `CLOG_AFTER_N(n, LEVEL, logger ID)`

```cpp
for ( int i = 0; i < 10; i++ )
{
    CLOG_AFTER_N( 6, ERROR, "map" ) << "This is a CLOG_AFTER_N ERROR log for map logger = " << i;
}
// 输出：
// This is a CLOG_AFTER_N ERROR log for map logger = 6
// This is a CLOG_AFTER_N ERROR log for map logger = 7
// This is a CLOG_AFTER_N ERROR log for map logger = 8
// This is a CLOG_AFTER_N ERROR log for map logger = 9
```

3. 从开始记录多少次
- `LOG_N_TIMES(n, LEVEL)`
- `CLOG_N_TIMES(n, LEVEL, logger ID)`

```cpp
for ( int i = 0; i < 10; i++ )
{
    CLOG_N_TIMES( 6, ERROR, "map" ) << "This is a CLOG_N_TIMES ERROR log for map logger = " << i;
}

// 输出：
// This is a CLOG_N_TIMES ERROR log for map logger = 0
// This is a CLOG_N_TIMES ERROR log for map logger = 1
// This is a CLOG_N_TIMES ERROR log for map logger = 2
// This is a CLOG_N_TIMES ERROR log for map logger = 3
// This is a CLOG_N_TIMES ERROR log for map logger = 4
// This is a CLOG_N_TIMES ERROR log for map logger = 5
```

## 类似printf的记录

需要c++11支持，与`printf`唯一的区别是所有的传入类型占位符为`%v`，并且消息格式输出中`%file`, `%func` `%line` and `%loc`就都不好用了。**不推荐**

- `info(const char*, const T&, const Args&...)`
- `warn(const char*, const T&, const Args&...)`
- `error(const char*, const T&, const Args&...)`
- `debug(const char*, const T&, const Args&...)`
- `fatal(const char*, const T&, const Args&...)`
- `trace(const char*, const T&, const Args&...)`
- `verbose(int vlevel, const char*, const T&, const Args&...)`

```cpp
el::Logger *mapLogger = el::Loggers::getLogger( "map" );
mapLogger->info( "Use map logger printf = %v | %v", "Hello map!", 1111111 );
// 输出：
// Use map logger printf = Hello map! | 1111111
```

## Verbose 日志

可以摆脱设置日志输出级别的限制，即脱离`DEBUG、INFO`等级别的门阀限制，可以指定单个文件的Vervose输出。

Verbose 日志可以理解为详细日志记录，可以记录比平时更多的信息。对故障排除非常有用。以下是详细的日志记录特定宏：

- `VLOG(verbose-level)`
- `CVLOG(verbose-level, logger ID)`

```cpp
 CVLOG( 0, "map" ) << "This is a CVLOG with level 0";
 // 输出：
 // This is a CVLOG with level 0
```

同样支持条件记录

- `VLOG_IF(condition, verbose-level)`
- `CVLOG_IF(condition, verbose-level, loggerID)`
- `VLOG_EVERY_N(n, verbose-level)`
- `CVLOG_EVERY_N(n, verbose-level, loggerID)`
- `VLOG_AFTER_N(n, verbose-level)`
- `CVLOG_AFTER_N(n, verbose-level, loggerID)`
- `VLOG_N_TIMES(n, verbose-level)`
- `CVLOG_N_TIMES(n, verbose-level, loggerID)`

Vervise日志的等级`0-9`，可以通过`Loggers::setVerboseLevel(base::type::VerboseLevel)`或者`Loggers::setVerboseLevel(int)`在程序中动态设置输出等级门阀，使用`Loggers::verboseLevel()`查看当前门阀等级，其中`0`等级的不会被门阀限制

# 注册新Logger

`Logger ID`是获取一个`Logger`的唯一标识符

默认情况下，系统初始化时会生成三个默认的记录器：

- Default logger (ID: `default`)
- Performance logger (ID: `performance`)
- Syslog logger (if `ELPP_SYSLOG` macro is defined) (ID: `syslog`)

如果想注册新的`Logger`：

```cpp
el::Logger* businessLogger = el::Loggers::getLogger("business");
```

即可获得一个`ID`为`business`的`Logger`

其中:

```cpp
el::Loggers::getLogger(const std::string& identity, bool registerIfNotAvailable = true)
```

函数的第二个参数默认为`true`，这种情况下`getLogger("business")`调用后，如果`business`的`Logger`存在，即返回一个`Logger`的对象指针，如果不存在，即创建一个相应的`Logger`并返回对象指针。

当通过`getLogger("business",false)`调用的时候，如果存在，返回对象指针，如果不存在，返回`nullptr`

**注：ID区分大小写**

# 注销Logger

除了上述3个默认创建的`Logger`无法注销之外

通过调用`el::Loggers::unregisterLogger("logger-id")`注销。

**注：小心使用该函数，有可能出现这种情况，你有一个第三方库使用了easylogging++，并且他使用的ID为myLog，当你自己注册一个Logger的时候也使用了ID为myLog（实际上并不是由你自己创建的，但是你误认为是你自己创建的），当你注销的时候有可能会导致程序崩溃**

# 查询Logger ID

通过调用函数

```cpp
el::Loggers::populateAllLoggerIds(std::vector<std::string>* v)
```

该函数会清空传入的`vector`并将`Logger ID`填充进去

> 如果定义`ELPP_SYSLOG`即可查看到`syslog`

# 设置库共享easylogging

...

# 额外功能

可以通过使用 `ELPP_FEATURE_ALL`来启用全部功能

## 性能跟踪

定义宏`ELPP_FEATURE_PERFORMANCE_TRACKING`，用于记录运行时间

- `TIMED_FUNC(obj-name)`：主要用来检测整个函数的性能，一般放在函数首行。
- `TIMED_SCOPE(obj-name, block-name)`：主要用来检测一定范围内的代码性能。
- `TIMED_BLOCK(obj-name, block-name)`：主要用来检测某一段代码块的性能。

例子如下：

```cpp
void performHeavyTask(int iter) {
   TIMED_FUNC(timerObj);
   // Some initializations
   // Some more heavy tasks
   usleep(5000);
   while (iter-- > 0) {
       TIMED_SCOPE(timerBlkObj, "heavy-iter");
       // Perform some heavy task in each iter
       usleep(10000);
   }
}
```

输出如下：

```cpp
06:22:31,368 INFO Executed [heavy-iter] in [10 ms]
06:22:31,379 INFO Executed [heavy-iter] in [10 ms]
06:22:31,389 INFO Executed [heavy-iter] in [10 ms]
06:22:31,399 INFO Executed [heavy-iter] in [10 ms]
06:22:31,409 INFO Executed [heavy-iter] in [10 ms]
06:22:31,419 INFO Executed [heavy-iter] in [10 ms]
06:22:31,429 INFO Executed [heavy-iter] in [10 ms]
06:22:31,440 INFO Executed [heavy-iter] in [10 ms]
06:22:31,450 INFO Executed [heavy-iter] in [10 ms]
06:22:31,460 INFO Executed [heavy-iter] in [10 ms]
06:22:31,460 INFO Executed [void performHeavyTask(int)] in [106 ms]
```

暂时没试出来

## 日志文件轮转

通过日志配置`Max_Log_File_Size`关键字进行配置

通过添加

```cpp
el::Loggers::addFlag( el::LoggingFlag::StrictLogFileSizeCheck );
```

使得配置生效。

通过注册`el::Helpers::installPreRollOutCallback(const PreRollOutCallback& handler)`使得其生效，其中`PreRollOutCallback`是`std::function<void(const char*, std::size_t)>`的typedef

一套简易的代码如下：

```cpp
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

static unsigned int idx;

void rolloutHandler( const char *filename, std::size_t size ) {
    // SHOULD NOT LOG ANYTHING HERE BECAUSE LOG FILE IS CLOSED!
    std::cout << "************** Rolling out [" << filename << "] because it reached [" << size << " bytes]"
              << std::endl;

    // BACK IT UP
    std::stringstream ss;
    system( "mkdir bak" );
    ss << "mv ./" << filename << " bak/log-backup.log";
    std::cout << ss.str() << std::endl;
    system( ss.str().c_str() );
}

int main( int, char ** ) {
    idx = 0;
    el::Configurations conf( "/home/cat/code/test/easylogging_usage/config/configLog.conf" );
    el::Loggers::reconfigureAllLoggers( conf );
    el::Loggers::addFlag( el::LoggingFlag::DisableApplicationAbortOnFatalLog );
    el::Loggers::addFlag( el::LoggingFlag::HierarchicalLogging );
    el::Loggers::addFlag( el::LoggingFlag::StrictLogFileSizeCheck );
    el::Helpers::installPreRollOutCallback( rolloutHandler );

    for ( int i = 0; i < 10000; ++i )
        LOG( INFO ) << "Test";

    el::Helpers::uninstallPreRollOutCallback();
    return 0;
}
```

**注：不能在该函数中使用LOG记录**

## 崩溃记录

添加宏`ELPP_FEATURE_CRASH_LOG`

`Easylogg++`为`GCC`编译器提供了处理意外崩溃的能力。默认情况下，这是活动的，可以通过定义宏`ELPP_DISABLE_default_CRASH_DHANDLING`来禁用。这样做是在告诉库不要处理任何崩溃。稍后，如果您希望自己处理崩溃，可以指定类型为`void func（int）`的崩溃处理程序，其中`int`是捕获信号的

以下的退出信号是能够被easylogging检测到的

- SIGABRT (需定义宏`ELPP_HANDLE_SIGABRT` )
- SIGFPE
- SIGILL
- SIGSEGV
- SIGINT

只有当定义了`ELPP_FEATURE_CRASH_LOG`宏后才打印堆栈信息

> 堆栈信息是通过`default`的logger id打印出来的

以下是一些有用的宏，您可以定义这些宏来更改行为

| 宏名称                                   | 描述                                                   |
| ------------------------------------- | ---------------------------------------------------- |
| `ELPP_DISABLE_DEFAULT_CRASH_HANDLING` | 禁用默认故障处理。                                            |
| `ELPP_HANDLE_SIGABRT`                 | 启用处理`SIGABRT`。默认情况下，这是禁用的，以防止在您希望中止时出现恼人的`CTRL+C`行为。 |

## 多线程

需要定义宏`ELPP_THREAD_SAFE`，确保easylogging是线程安全的，一用就挂！！！！

## STL日志

定义宏`ELPP_STL_LOGGING`

如前所述，使用easylogging++，您可以记录STL模板，包括大多数容器。为此，您需要定义`ELPP_STL_LOGGING`宏。这允许包含所有必要的标头并定义所有必要的函数。为了提高性能，容器最多只能记录100个条目。这种行为可以通过更改头文件（`base:：consts::kMaxLogPerContainer`）来改变，但不建议这样做，因为为了进行日志记录，写入程序必须遍历每个条目，这可能会导致延迟。但是，如果您并不真正关心性能，您可能会更改此值。

## Boost日志

定义宏`ELPP_BOOST_LOGGING`
