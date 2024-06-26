//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Gen3Moveit
{
    public class PoseEstimationServiceRequest : Message
    {
        public const string RosMessageName = "gen3_moveit/PoseEstimationService";

        public Sensor.Image image;

        public PoseEstimationServiceRequest()
        {
            this.image = new Sensor.Image();
        }

        public PoseEstimationServiceRequest(Sensor.Image image)
        {
            this.image = image;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(image.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.image.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "PoseEstimationServiceRequest: " +
            "\nimage: " + image.ToString();
        }
    }
}
