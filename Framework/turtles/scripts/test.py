import rospy
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_gateway

rospy.init_node('advertise')
rospy.wait_for_service('/gateway/advertise')
advertise_service=rospy.ServiceProxy('/gateway/advertise', gateway_srvs.Advertise)

req = gateway_srvs.AdvertiseRequest()

rule = gateway_msgs.Rule()
rule.name = '/chatter'
rule.type = gateway_msgs.ConnectionType.PUBLISHER
rule.node = '/talker'

rospy.loginfo("Advertise : [%s,%s,%s]."%(rule.type, rule.name, rule.node))

req.rules.append(rule)
resp = advertise_service(req)

if resp.result != 0:
    rospy.logerr("Advertise : %s"%resp.error_message)
