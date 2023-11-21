#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from firebase import firebase

firebaseConfig = {
  "apiKey": "AIzaSyAYAqXoiL0L-aBW9RkqLGKZbmWxB0jQPS0",
  "authDomain": "agni-e2d93.firebaseapp.com",
  "databaseURL": "https://agni-e2d93-default-rtdb.firebaseio.com",
  "projectId": "agni-e2d93",
  "storageBucket": "agni-e2d93.appspot.com",
  "messagingSenderId": "190886177892",
  "appId": "1:190886177892:web:39faf73be52dfc40428371",
  "measurementId": "G-QZ9EV795NZ"
}

# Create an instance of the Firebase class
firebase_app = firebase.FirebaseApplication("https://agni-e2d93-default-rtdb.firebaseio.com", None)
def gps_location_callback(msg):
    try:
        # Assuming msg.data is a string with latitude and longitude separated by a comma
        latitude, longitude = map(float, msg.data.split(','))

        # Create a dictionary with the data
        data_to_update = {"latitude": latitude, "longitude": longitude}

        # Update data under the "latest" node in the Firebase Realtime Database
        firebase_app.put("/current_location", "latest", data_to_update)

        rospy.loginfo("Data updated successfully for key: latest")

    except Exception as e:
        rospy.logerr("Error processing GPS location message: %s", str(e))

def main():
    rospy.init_node('firebase_gps_publisher', anonymous=True)

    # Subscribe to the /gps_location topic
    rospy.Subscriber("/gps_location", String, gps_location_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
