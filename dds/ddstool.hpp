// clang-format off
#pragma once
#include <iostream>
#include <stdio.h>
#include <dlfcn.h>
#include <functional>
#include "initialization.hpp"
#include <memory>
class DDSTool
{
public:
using func_ptr = bool(*)(const char*, uint64_t , const void*);
using DDSLoadFunc = bool(*)();
using DDSCreatePubFunc = bool(*)(const std::string& group, const std::string& topic);
using DDSPubFunc = bool(*)(const std::string& topic,const std::string& info, const std::vector<char>& data);
using DDSPubDFunc = bool(*)(std::weak_ptr<void> wirter,const std::string& info, const std::vector<char>& data);
using DDSCreateSubFunc = bool(*)(const std::string& group, const std::string& topic,std::function<void(const std::string&, const std::vector<char>&)> cb);
using DDSCreateRequestFunc = bool(*)(const std::string& topic);
using DDSCreateResponseFunc = bool(*)(const std::string& topic,std::function<bool(const std::string&, const std::vector<char>&,std::string&, std::vector<char>&)> cb);
using DDSRequestFunc = bool(*) (const std::string& topic,const std::string& reqInfo, const std::vector<char>& reqData,std::string& respInfo, std::vector<char>& respData);

DDSTool()
{
    try
    {
        std::string nav3dInstallPathStr;
        bool isGetPath = RoboSLAM3::Init::GetInstallPath(nav3dInstallPathStr);
        if (!isGetPath) {
            return;
        }
        std::string libdds_path=nav3dInstallPathStr+"/lib/libdds_util.so";
        std::cout << "libdds_util.so path: " << libdds_path << std::endl;
        handle = dlopen(libdds_path.c_str(), RTLD_LAZY);
        if(handle == nullptr)
        {
            std::cout << "libdds_util.so is not exist, please put the lib in the same path as the executable" << std::endl;
        }
        createSub_ = (DDSCreateSubFunc)dlsym(handle, "cxx_createSub");
        createPub_ = (DDSCreatePubFunc)dlsym(handle, "cxx_createPub");
        pub_ = (DDSPubFunc)dlsym(handle, "cxx_pub");
        pubD_ = (DDSPubDFunc)dlsym(handle, "cxx_pubD");
        load_ = (DDSLoadFunc)dlsym(handle, "cxx_load");
        unload_ = (DDSLoadFunc)dlsym(handle, "cxx_unload");
        reset_ = (DDSLoadFunc)dlsym(handle, "cxx_reset");
        createResponse_ = (DDSCreateResponseFunc)dlsym(handle, "cxx_createResponse");
        createRequest_ = (DDSCreateRequestFunc)dlsym(handle,"cxx_createRequest");
        request_ = (DDSRequestFunc)dlsym(handle,"cxx_request");
    }
    catch(...)
    {
        std::cout << "dlopen or dlsym core" << std::endl;
    }
}

~DDSTool()
{
    unload();
    dlclose(handle);
}

bool load()
{
    return load_ && load_();
}

bool unload()
{
    return unload_ && unload_();
}

bool createPub(const std::string& group, const std::string& topic)
{
    return createPub_ && createPub_(group,topic);
}

bool pub(const std::string& topic,const std::string& info, const std::vector<char>& data)
{
    return pub_ && pub_(topic,info,data);
}


bool createSub(const std::string& group, const std::string& topic,std::function<void(const std::string&, const std::vector<char>&)> cb)
{
    return createSub_ && createSub_(group,topic,cb);
}

bool relase()
{
    return relase_ && relase();
}


bool createRequest(const std::string& topic)
{
    return createRequest_ && createRequest_(topic);
}

bool createResponse(const std::string& topic,std::function<bool(const std::string&, const std::vector<char>&,std::string&, std::vector<char>&)> cb)
{
    return createResponse_ && createResponse_(topic,cb);
}

bool request(const std::string& topic,const std::string& reqInfo, const std::vector<char>& reqData,std::string& respInfo, std::vector<char>& respData)
{
    return request_ && request_(topic,reqInfo,reqData,respInfo,respData);
}

private:
    void* handle = nullptr;
    DDSLoadFunc relase_ = nullptr;
    DDSLoadFunc reset_ = nullptr;
    DDSLoadFunc load_ = nullptr;
    DDSLoadFunc unload_ = nullptr;
    DDSPubFunc pub_ = nullptr;
    DDSPubDFunc pubD_ = nullptr;
    DDSCreateSubFunc createSub_ = nullptr;
    DDSCreatePubFunc createPub_ = nullptr; 
    DDSCreateRequestFunc createRequest_ = nullptr;
    DDSCreateResponseFunc createResponse_ = nullptr; 
    DDSRequestFunc request_ = nullptr;
};
