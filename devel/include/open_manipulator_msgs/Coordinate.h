// Generated by gencpp from file open_manipulator_msgs/Coordinate.msg
// DO NOT EDIT!


#ifndef OPEN_MANIPULATOR_MSGS_MESSAGE_COORDINATE_H
#define OPEN_MANIPULATOR_MSGS_MESSAGE_COORDINATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace open_manipulator_msgs
{
template <class ContainerAllocator>
struct Coordinate_
{
  typedef Coordinate_<ContainerAllocator> Type;

  Coordinate_()
    : start_time()
    , msg_seq(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , distance(0.0)  {
    }
  Coordinate_(const ContainerAllocator& _alloc)
    : start_time()
    , msg_seq(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , distance(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef uint16_t _msg_seq_type;
  _msg_seq_type msg_seq;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> const> ConstPtr;

}; // struct Coordinate_

typedef ::open_manipulator_msgs::Coordinate_<std::allocator<void> > Coordinate;

typedef boost::shared_ptr< ::open_manipulator_msgs::Coordinate > CoordinatePtr;
typedef boost::shared_ptr< ::open_manipulator_msgs::Coordinate const> CoordinateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::open_manipulator_msgs::Coordinate_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::open_manipulator_msgs::Coordinate_<ContainerAllocator1> & lhs, const ::open_manipulator_msgs::Coordinate_<ContainerAllocator2> & rhs)
{
  return lhs.start_time == rhs.start_time &&
    lhs.msg_seq == rhs.msg_seq &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.distance == rhs.distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::open_manipulator_msgs::Coordinate_<ContainerAllocator1> & lhs, const ::open_manipulator_msgs::Coordinate_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace open_manipulator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "61deae5df730e150ae50b08d27dbc08d";
  }

  static const char* value(const ::open_manipulator_msgs::Coordinate_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x61deae5df730e150ULL;
  static const uint64_t static_value2 = 0xae50b08d27dbc08dULL;
};

template<class ContainerAllocator>
struct DataType< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "open_manipulator_msgs/Coordinate";
  }

  static const char* value(const ::open_manipulator_msgs::Coordinate_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time start_time\n"
"uint16 msg_seq\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 distance\n"
;
  }

  static const char* value(const ::open_manipulator_msgs::Coordinate_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.start_time);
      stream.next(m.msg_seq);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Coordinate_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::open_manipulator_msgs::Coordinate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::open_manipulator_msgs::Coordinate_<ContainerAllocator>& v)
  {
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "msg_seq: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.msg_seq);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "distance: ";
    Printer<double>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPEN_MANIPULATOR_MSGS_MESSAGE_COORDINATE_H
