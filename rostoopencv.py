class image_converter:
  def __init__(self):
   self.image_pub = rospy.Publisher("image_topic_2",Image)

   self.bridge = CvBridge()
   self.image_sub = rospy.Subscriber("image_topic",Image,self.cvcallback)
   
  def cvcallback(self,data):
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
  # Convert BGR to HSV
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  
  
  # define range of red color in HSV
  lower_range = np.array([110,50,50]) #need to find
  upper_range = np.array([130,255,255]) #need to find
  
  # Threshold the HSV image to get only blue colors
  mask = cv2.inRange(hsv, lower_range, upper_range)
  
  opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
  x ,y, w, h = cv2.boundingRect(opening)
  cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),3)
  centerX = x+w/2
  centerY = y+h/2
  
  cv2.imshow("Image window", cv_image)
  
  cv2.waitKey(3)
  
  #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
 
if __name__ == '__main__':
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()
  
  
