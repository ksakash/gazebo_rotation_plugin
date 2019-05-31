// Generated by gencpp from file uuv_control_msgs/GetMBSMControllerParamsResponse.msg
// DO NOT EDIT!


#ifndef UUV_CONTROL_MSGS_MESSAGE_GETMBSMCONTROLLERPARAMSRESPONSE_H
#define UUV_CONTROL_MSGS_MESSAGE_GETMBSMCONTROLLERPARAMSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace uuv_control_msgs
{
template <class ContainerAllocator>
struct GetMBSMControllerParamsResponse_
{
  typedef GetMBSMControllerParamsResponse_<ContainerAllocator> Type;

  GetMBSMControllerParamsResponse_()
    : lambda()
    , rho_constant()
    , k()
    , c()
    , adapt_slope()
    , rho_0()
    , drift_prevent(0.0)  {
    }
  GetMBSMControllerParamsResponse_(const ContainerAllocator& _alloc)
    : lambda(_alloc)
    , rho_constant(_alloc)
    , k(_alloc)
    , c(_alloc)
    , adapt_slope(_alloc)
    , rho_0(_alloc)
    , drift_prevent(0.0)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _lambda_type;
  _lambda_type lambda;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _rho_constant_type;
  _rho_constant_type rho_constant;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _k_type;
  _k_type k;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _c_type;
  _c_type c;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _adapt_slope_type;
  _adapt_slope_type adapt_slope;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _rho_0_type;
  _rho_0_type rho_0;

   typedef double _drift_prevent_type;
  _drift_prevent_type drift_prevent;





  typedef boost::shared_ptr< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetMBSMControllerParamsResponse_

typedef ::uuv_control_msgs::GetMBSMControllerParamsResponse_<std::allocator<void> > GetMBSMControllerParamsResponse;

typedef boost::shared_ptr< ::uuv_control_msgs::GetMBSMControllerParamsResponse > GetMBSMControllerParamsResponsePtr;
typedef boost::shared_ptr< ::uuv_control_msgs::GetMBSMControllerParamsResponse const> GetMBSMControllerParamsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace uuv_control_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/home/ksakash/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/home/ksakash/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg'], 'uuv_control_msgs': ['/home/ksakash/uuv_ws/src/uuv_simulator/uuv_control/uuv_control_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7293aecc8487ffe3e998814d65aa6940";
  }

  static const char* value(const ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7293aecc8487ffe3ULL;
  static const uint64_t static_value2 = 0xe998814d65aa6940ULL;
};

template<class ContainerAllocator>
struct DataType< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uuv_control_msgs/GetMBSMControllerParamsResponse";
  }

  static const char* value(const ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] lambda\n"
"float64[] rho_constant\n"
"float64[] k\n"
"float64[] c\n"
"float64[] adapt_slope\n"
"float64[] rho_0\n"
"float64 drift_prevent\n"
"\n"
;
  }

  static const char* value(const ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lambda);
      stream.next(m.rho_constant);
      stream.next(m.k);
      stream.next(m.c);
      stream.next(m.adapt_slope);
      stream.next(m.rho_0);
      stream.next(m.drift_prevent);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetMBSMControllerParamsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::uuv_control_msgs::GetMBSMControllerParamsResponse_<ContainerAllocator>& v)
  {
    s << indent << "lambda[]" << std::endl;
    for (size_t i = 0; i < v.lambda.size(); ++i)
    {
      s << indent << "  lambda[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.lambda[i]);
    }
    s << indent << "rho_constant[]" << std::endl;
    for (size_t i = 0; i < v.rho_constant.size(); ++i)
    {
      s << indent << "  rho_constant[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.rho_constant[i]);
    }
    s << indent << "k[]" << std::endl;
    for (size_t i = 0; i < v.k.size(); ++i)
    {
      s << indent << "  k[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.k[i]);
    }
    s << indent << "c[]" << std::endl;
    for (size_t i = 0; i < v.c.size(); ++i)
    {
      s << indent << "  c[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.c[i]);
    }
    s << indent << "adapt_slope[]" << std::endl;
    for (size_t i = 0; i < v.adapt_slope.size(); ++i)
    {
      s << indent << "  adapt_slope[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.adapt_slope[i]);
    }
    s << indent << "rho_0[]" << std::endl;
    for (size_t i = 0; i < v.rho_0.size(); ++i)
    {
      s << indent << "  rho_0[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.rho_0[i]);
    }
    s << indent << "drift_prevent: ";
    Printer<double>::stream(s, indent + "  ", v.drift_prevent);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UUV_CONTROL_MSGS_MESSAGE_GETMBSMCONTROLLERPARAMSRESPONSE_H
