/* Auto-generated by genmsg_cpp for file /home/ros/rosbuild_ws/prlite-pc/net_485net_id_handler/srv/SearchID.srv */
#ifndef NET_485NET_ID_HANDLER_SERVICE_SEARCHID_H
#define NET_485NET_ID_HANDLER_SERVICE_SEARCHID_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace net_485net_id_handler
{
template <class ContainerAllocator>
struct SearchIDRequest_ {
  typedef SearchIDRequest_<ContainerAllocator> Type;

  SearchIDRequest_()
  : type()
  , desc()
  {
  }

  SearchIDRequest_(const ContainerAllocator& _alloc)
  : type(_alloc)
  , desc(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  type;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _desc_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  desc;


  typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SearchIDRequest
typedef  ::net_485net_id_handler::SearchIDRequest_<std::allocator<void> > SearchIDRequest;

typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDRequest> SearchIDRequestPtr;
typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDRequest const> SearchIDRequestConstPtr;



template <class ContainerAllocator>
struct SearchIDResponse_ {
  typedef SearchIDResponse_<ContainerAllocator> Type;

  SearchIDResponse_()
  : res(0)
  {
  }

  SearchIDResponse_(const ContainerAllocator& _alloc)
  : res(0)
  {
  }

  typedef uint8_t _res_type;
  uint8_t res;


  typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SearchIDResponse
typedef  ::net_485net_id_handler::SearchIDResponse_<std::allocator<void> > SearchIDResponse;

typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDResponse> SearchIDResponsePtr;
typedef boost::shared_ptr< ::net_485net_id_handler::SearchIDResponse const> SearchIDResponseConstPtr;


struct SearchID
{

typedef SearchIDRequest Request;
typedef SearchIDResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SearchID
} // namespace net_485net_id_handler

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "83d582892de6e5f46a5d302cfff9f311";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x83d582892de6e5f4ULL;
  static const uint64_t static_value2 = 0x6a5d302cfff9f311ULL;
};

template<class ContainerAllocator>
struct DataType< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "net_485net_id_handler/SearchIDRequest";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string type\n\
string desc\n\
\n\
";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f4cf94077d46a7ac28e1686e63bb1b07";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf4cf94077d46a7acULL;
  static const uint64_t static_value2 = 0x28e1686e63bb1b07ULL;
};

template<class ContainerAllocator>
struct DataType< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "net_485net_id_handler/SearchIDResponse";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 res\n\
\n\
\n\
";
  }

  static const char* value(const  ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::net_485net_id_handler::SearchIDRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.type);
    stream.next(m.desc);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SearchIDRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::net_485net_id_handler::SearchIDResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.res);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SearchIDResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<net_485net_id_handler::SearchID> {
  static const char* value() 
  {
    return "78e222123899a19f1660debb8b9dde08";
  }

  static const char* value(const net_485net_id_handler::SearchID&) { return value(); } 
};

template<>
struct DataType<net_485net_id_handler::SearchID> {
  static const char* value() 
  {
    return "net_485net_id_handler/SearchID";
  }

  static const char* value(const net_485net_id_handler::SearchID&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "78e222123899a19f1660debb8b9dde08";
  }

  static const char* value(const net_485net_id_handler::SearchIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<net_485net_id_handler::SearchIDRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "net_485net_id_handler/SearchID";
  }

  static const char* value(const net_485net_id_handler::SearchIDRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "78e222123899a19f1660debb8b9dde08";
  }

  static const char* value(const net_485net_id_handler::SearchIDResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<net_485net_id_handler::SearchIDResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "net_485net_id_handler/SearchID";
  }

  static const char* value(const net_485net_id_handler::SearchIDResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NET_485NET_ID_HANDLER_SERVICE_SEARCHID_H

