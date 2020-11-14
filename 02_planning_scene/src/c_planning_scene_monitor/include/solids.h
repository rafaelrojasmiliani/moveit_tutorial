
#ifndef SOLIDS_H
#define SOLIDS_H
#include<shape_msgs/SolidPrimitive.h>
#include<moveit_msgs/CollisionObject.h>
shape_msgs::SolidPrimitive get_primitive(const std::string& _type);
moveit_msgs::CollisionObject get_colision_object(const std::string& _header_frame, const shape_msgs::SolidPrimitive& _shape, double _x, double _y, double _z);
#endif
