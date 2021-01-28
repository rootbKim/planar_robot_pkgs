// Generated by gencpp from file planar_robot_pkgs/Planar_CTCMsg.msg
// DO NOT EDIT!


#ifndef PLANAR_ROBOT_PKGS_MESSAGE_PLANAR_CTCMSG_H
#define PLANAR_ROBOT_PKGS_MESSAGE_PLANAR_CTCMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace planar_robot_pkgs
{
template <class ContainerAllocator>
struct Planar_CTCMsg_
{
  typedef Planar_CTCMsg_<ContainerAllocator> Type;

  Planar_CTCMsg_()
    : TF(0)
    , Des_X_y(0.0)
    , Des_X_z(0.0)
    , Des_XDot_y(0.0)
    , Des_XDot_z(0.0)
    , Des_XDDot_y(0.0)
    , Des_XDDot_z(0.0)
    , Kp0(0.0)
    , Kp1(0.0)
    , Kv0(0.0)
    , Kv1(0.0)  {
    }
  Planar_CTCMsg_(const ContainerAllocator& _alloc)
    : TF(0)
    , Des_X_y(0.0)
    , Des_X_z(0.0)
    , Des_XDot_y(0.0)
    , Des_XDot_z(0.0)
    , Des_XDDot_y(0.0)
    , Des_XDDot_z(0.0)
    , Kp0(0.0)
    , Kp1(0.0)
    , Kv0(0.0)
    , Kv1(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _TF_type;
  _TF_type TF;

   typedef float _Des_X_y_type;
  _Des_X_y_type Des_X_y;

   typedef float _Des_X_z_type;
  _Des_X_z_type Des_X_z;

   typedef float _Des_XDot_y_type;
  _Des_XDot_y_type Des_XDot_y;

   typedef float _Des_XDot_z_type;
  _Des_XDot_z_type Des_XDot_z;

   typedef float _Des_XDDot_y_type;
  _Des_XDDot_y_type Des_XDDot_y;

   typedef float _Des_XDDot_z_type;
  _Des_XDDot_z_type Des_XDDot_z;

   typedef float _Kp0_type;
  _Kp0_type Kp0;

   typedef float _Kp1_type;
  _Kp1_type Kp1;

   typedef float _Kv0_type;
  _Kv0_type Kv0;

   typedef float _Kv1_type;
  _Kv1_type Kv1;





  typedef boost::shared_ptr< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> const> ConstPtr;

}; // struct Planar_CTCMsg_

typedef ::planar_robot_pkgs::Planar_CTCMsg_<std::allocator<void> > Planar_CTCMsg;

typedef boost::shared_ptr< ::planar_robot_pkgs::Planar_CTCMsg > Planar_CTCMsgPtr;
typedef boost::shared_ptr< ::planar_robot_pkgs::Planar_CTCMsg const> Planar_CTCMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace planar_robot_pkgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'planar_robot_pkgs': ['/home/jiyong/catkin_ws/src/planar_robot_pkgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8457c1274f7beb15b25f10a1340be49";
  }

  static const char* value(const ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8457c1274f7beb1ULL;
  static const uint64_t static_value2 = 0x5b25f10a1340be49ULL;
};

template<class ContainerAllocator>
struct DataType< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "planar_robot_pkgs/Planar_CTCMsg";
  }

  static const char* value(const ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 TF\n\
float32 Des_X_y\n\
float32 Des_X_z\n\
float32 Des_XDot_y\n\
float32 Des_XDot_z\n\
float32 Des_XDDot_y\n\
float32 Des_XDDot_z\n\
float32 Kp0\n\
float32 Kp1\n\
float32 Kv0\n\
float32 Kv1\n\
";
  }

  static const char* value(const ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.TF);
      stream.next(m.Des_X_y);
      stream.next(m.Des_X_z);
      stream.next(m.Des_XDot_y);
      stream.next(m.Des_XDot_z);
      stream.next(m.Des_XDDot_y);
      stream.next(m.Des_XDDot_z);
      stream.next(m.Kp0);
      stream.next(m.Kp1);
      stream.next(m.Kv0);
      stream.next(m.Kv1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Planar_CTCMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::planar_robot_pkgs::Planar_CTCMsg_<ContainerAllocator>& v)
  {
    s << indent << "TF: ";
    Printer<int32_t>::stream(s, indent + "  ", v.TF);
    s << indent << "Des_X_y: ";
    Printer<float>::stream(s, indent + "  ", v.Des_X_y);
    s << indent << "Des_X_z: ";
    Printer<float>::stream(s, indent + "  ", v.Des_X_z);
    s << indent << "Des_XDot_y: ";
    Printer<float>::stream(s, indent + "  ", v.Des_XDot_y);
    s << indent << "Des_XDot_z: ";
    Printer<float>::stream(s, indent + "  ", v.Des_XDot_z);
    s << indent << "Des_XDDot_y: ";
    Printer<float>::stream(s, indent + "  ", v.Des_XDDot_y);
    s << indent << "Des_XDDot_z: ";
    Printer<float>::stream(s, indent + "  ", v.Des_XDDot_z);
    s << indent << "Kp0: ";
    Printer<float>::stream(s, indent + "  ", v.Kp0);
    s << indent << "Kp1: ";
    Printer<float>::stream(s, indent + "  ", v.Kp1);
    s << indent << "Kv0: ";
    Printer<float>::stream(s, indent + "  ", v.Kv0);
    s << indent << "Kv1: ";
    Printer<float>::stream(s, indent + "  ", v.Kv1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLANAR_ROBOT_PKGS_MESSAGE_PLANAR_CTCMSG_H