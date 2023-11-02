/**
 * @file exception.h
 * @brief 异常类
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
#define ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <string>
#include <stdexcept>

namespace rokae {

/**
 * @class Exception
 * @brief 运行中异常基类
 */
 class Exception : public std::exception {
  public:
   /**
    * @brief constructor
    * @param detailEN English ver.
    * @param detailCN Chinese ver.
    */
   explicit Exception(const std::string &detailEN, const std::string &detailCN);
   /**
    * @brief 异常信息
    */
   const char* what() const noexcept override;

  protected:
   const std::string detailEN_; ///< English ver.
   const std::string detailCN_; ///< Chinese ver.
 };

/**
 * @class NetworkException
 * @brief 网络异常
 */
 class NetworkException final : public Exception {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit NetworkException(const std::string &en, const std::string &cn);
 };

/**
 * @class ArgumentException
 * @brief 参数错误异常
 */
 class ArgumentException : public Exception {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit ArgumentException(const std::string &en, const std::string &cn);
 };
/**
 * @class ExecutionException
 * @brief 操作执行失败异常
 */
 class ExecutionException : public Exception {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit ExecutionException (const std::string &en, const std::string &cn);
 };

/**
 * @class ProtocolException
 * @brief 解析控制器消息失败异常, 可能由于SDK版本与控制器版本不匹配
 */
 class ProtocolException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit ProtocolException(const std::string &en, const std::string &cn);
 };

/**
 * @class InvalidOperationException
 * @brief 操作被控制器拒绝
 */
 class InvalidOperationException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit InvalidOperationException(const std::string &en, const std::string &cn);
 };

/**
 * @class RealtimeControlException
 * @brief 实时模式错误
 */
 class RealtimeControlException : public Exception {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit RealtimeControlException(const std::string &en, const std::string &cn);
 };

/**
 * @class RealtimeMotionException
 * @brief 实时模式运动错误
 */
 class RealtimeMotionException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit RealtimeMotionException(const std::string &en, const std::string &cn);
 };

/**
 * @class RealtimeStateException
 * @brief 实时模式状态错误
 */
 class RealtimeStateException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit RealtimeStateException(const std::string &en, const std::string &cn);
 };
/**
 * @class RealtimeParameterException
 * @brief 实时模式参数错误
 */
 class RealtimeParameterException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param en English ver.
    * @param cn Chinese ver.
    */
   explicit RealtimeParameterException(const std::string &en, const std::string &cn);
 };
}// namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
