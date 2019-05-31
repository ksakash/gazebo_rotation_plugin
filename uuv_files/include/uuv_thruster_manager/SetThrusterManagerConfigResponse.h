// Generated by gencpp from file uuv_thruster_manager/SetThrusterManagerConfigResponse.msg
// DO NOT EDIT!


#ifndef UUV_THRUSTER_MANAGER_MESSAGE_SETTHRUSTERMANAGERCONFIGRESPONSE_H
#define UUV_THRUSTER_MANAGER_MESSAGE_SETTHRUSTERMANAGERCONFIGRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace uuv_thruster_manager
{
template <class ContainerAllocator>
struct SetThrusterManagerConfigResponse_
{
  typedef SetThrusterManagerConfigResponse_<ContainerAllocator> Type;

  SetThrusterManagerConfigResponse_()
    : success(false)  {
    }
  SetThrusterManagerConfigResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetThrusterManagerConfigResponse_

typedef ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<std::allocator<void> > SetThrusterManagerConfigResponse;

typedef boost::shared_ptr< ::uuv_thruster_manager::SetThrusterManagerConfigResponse > SetThrusterManagerConfigResponsePtr;
typedef boost::shared_ptr< ::uuv_thruster_manager::SetThrusterManagerConfigResponse const> SetThrusterManagerConfigResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace uuv_thruster_manager

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/ksakash/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uuv_thruster_manager/SetThrusterManagerConfigResponse";
  }

  static const char* value(const ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetThrusterManagerConfigResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::uuv_thruster_manager::SetThrusterManagerConfigResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UUV_THRUSTER_MANAGER_MESSAGE_SETTHRUSTERMANAGERCONFIGRESPONSE_H
