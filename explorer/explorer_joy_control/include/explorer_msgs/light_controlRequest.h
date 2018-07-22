// Generated by gencpp from file explorer_msgs/light_controlRequest.msg
// DO NOT EDIT!


#ifndef EXPLORER_MSGS_MESSAGE_LIGHT_CONTROLREQUEST_H
#define EXPLORER_MSGS_MESSAGE_LIGHT_CONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace explorer_msgs
{
template <class ContainerAllocator>
struct light_controlRequest_
{
  typedef light_controlRequest_<ContainerAllocator> Type;

  light_controlRequest_()
    : light_on(false)  {
    }
  light_controlRequest_(const ContainerAllocator& _alloc)
    : light_on(false)  {
  (void)_alloc;
    }



   typedef uint8_t _light_on_type;
  _light_on_type light_on;




  typedef boost::shared_ptr< ::explorer_msgs::light_controlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::explorer_msgs::light_controlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct light_controlRequest_

typedef ::explorer_msgs::light_controlRequest_<std::allocator<void> > light_controlRequest;

typedef boost::shared_ptr< ::explorer_msgs::light_controlRequest > light_controlRequestPtr;
typedef boost::shared_ptr< ::explorer_msgs::light_controlRequest const> light_controlRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::explorer_msgs::light_controlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace explorer_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'explorer_msgs': ['/home/hdh/catkin_ws/src/explorer/explorer_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::explorer_msgs::light_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::explorer_msgs::light_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::explorer_msgs::light_controlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3026467b703261111d596d8ddecb028c";
  }

  static const char* value(const ::explorer_msgs::light_controlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3026467b70326111ULL;
  static const uint64_t static_value2 = 0x1d596d8ddecb028cULL;
};

template<class ContainerAllocator>
struct DataType< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "explorer_msgs/light_controlRequest";
  }

  static const char* value(const ::explorer_msgs::light_controlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool light_on\n\
";
  }

  static const char* value(const ::explorer_msgs::light_controlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.light_on);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct light_controlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::explorer_msgs::light_controlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::explorer_msgs::light_controlRequest_<ContainerAllocator>& v)
  {
    s << indent << "light_on: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.light_on);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXPLORER_MSGS_MESSAGE_LIGHT_CONTROLREQUEST_H