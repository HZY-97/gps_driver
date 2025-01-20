

# DDS动态库使用说明

## 使用方法

首先将ddstool.hpp 代码加入到需使用dds动态库的工程目录下，并在自己的代码里include进去，编译好可执行程序后，在运行时需将libdds_util.so放在可执行程序的**同级目录**下。

## DDS动态库API详细介绍

ddstool.hpp该文件共有以下API，详细介绍分别如下：

```c++
//brief：初始化dds相关配置，调用发布或订阅之前调用
//return value：bool类型，初始化成功或失败
bool DDSTool::load()；

//brief：释放dds相关配置，调用发布或订阅之后调用
//return value：bool类型，释放成功或失败
bool DDSTool::unload()；

//brief：向指定的主题发布消息
//input：topic： 向该主题发布消息 
//    	 info： 发布的数据的信息，自定义的数据信息，可任意定义 
//		 data： 要发布的数据
//return value：bool类型，消息发布成功或失败
bool DDSTool::pub(const std::string& topic,const std::string& info, const std::vector<char>& data)；
    
//brief：创建订阅者
//input：group： 分组
//    	 topic： 订阅的主题
//       cb：订阅回调函数
//return value：bool类型，消息接收成功或失败
bool DDSTool::createSub(const std::string& group, const std::string& topic,std::function<void(const std::string&, const std::vector<char>&)> cb)；

//brief：创建发布者
//input：group： 分组
//    	 topic： 发布的主题
//return value：bool类型，消息接收成功或发布
bool DDSTool::createPub(const std::string& group, const std::string& topic)；

    
//brief：清空发布订阅List
//return value：bool类型，是否成功清空
bool DDSTool::release()；
    
//brief：创建发布
bool createRequest(const std::string& topic)；

//brief：创建响应
bool createResponse(const std::string& topic,std::function<bool(const std::string&, const std::vector<char>&,std::string&, std::vector<char>&)> cb)；

//brief： 向server端发送请求 
bool request(const std::string& topic,const std::string& reqInfo, const std::vector<char>& reqData,std::string& respInfo, std::vector<char>& respData)
```



## 使用示例

发布消息使用参考示例如下：

```c++
#include <memory>
#include "ddstool.hpp"
#include "unistd.h"
#include <string.h>
#include <iostream>

using namespace std;

int main(){
	std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>();
    std::string group = "group";
    std::string topic = "test";
    std::string info = "test info";
    std::string datastr = "this is a publish demo"; 
    std::vector<char> data{datastr.c_str(), datastr.c_str() + datastr.size()};        
	dds_ptr->load(); // 初始化dds
    dds_ptr->createPub(group,topic);
	
	int count = 0;
	while(1)
	{
        auto res = dds_ptr->pub(topic,std::to_string(count++),data);
        std::cout << "res:" << res << std::endl;
		sleep(1);
	}
	dds_ptr->unload();  
}
```

订阅消息使用参考示例如下：

```c++
#include <memory>
#include "ddstool.hpp"
#include "unistd.h"
#include <string.h>
#include <iostream>

using namespace std;

int main(){
	std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>(); 
    std::string group = "group";
    std::string topic = "test";      
	dds_ptr->load(); // 初始化dds
    dds_ptr->createSub(group,topic,[&](const std::string& topic, const std::vector<char>&data)
									{
										std::cout << "topic: " << topic<< std::endl;
                                        std::cout << "data: " << data.data() << std::endl;
									});
	
	while(1)
	{
		sleep(1);
	}
	dds_ptr->unload(); 
}



```



请求响应使用参考示例：

```c++
#include <memory>
#include "ddstool.hpp"
#include "unistd.h"
#include <string.h>
#include <iostream>

using namespace std;

int main(){
	std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>(); // 创建DDSTool对象
	std::string topic = "test";
	std::string reqinfo = "req";
	std::string repinfo;
	std::vector<char> reqdata,resdata;
	dds_ptr->load(); // 初始化dds
	dds_ptr->createRequest(topic);
	sleep(0.5);
	dds_ptr->request(topic,reqinfo,reqdata,repinfo,resdata);
	
	while(1)
	{
		sleep(1);
	}
	dds_ptr->unload();  
}



```

```c++
#include <memory>
#include "ddstool.hpp"
#include "unistd.h"
#include <string.h>
#include <iostream>

using namespace std;

int main(){
	std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>(); 
	std::string topic = "test";
	dds_ptr->load(); // 初始化dds
	dds_ptr->createResponse(topic,[&](const std::string& str1, const std::vector<char>&data1,std::string& str2, std::vector<char>&data2)
									{
										std::cout << "str1 " << str1<< std::endl;
										std::cout << "str2 " << str2<< std::endl;
										return true;
									});
	
	while(1)
	{
		sleep(1);
	}
	dds_ptr->unload(); 
}



```

