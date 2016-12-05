class image_converter:
  def __init__(self):
   self.image_pub = rospy.Publisher("image_topic_2",Image)

   self.bridge = CvBridge()
   self.image_sub = rospy.Subscriber("image_topic",Image,self.cvcallback)
   
  def cvcallback(self,data):
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
  # check 
      
  cv2.imshow("Image window", cv_image)
  
  cv2.waitKey(3)
  
  #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
 
if __name__ == '__main__':
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()
  
  
