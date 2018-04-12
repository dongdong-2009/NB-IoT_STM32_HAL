# mesh_trans
数据采集和传输
## 设备编号
每个设备应该有个唯一的编号，注册在服务端，作为入网许可。设备编号暂定为两个字节，以后可以扩展.例如：  
```
0xee,0x13
```
## 采集数据格式
各种采集节点都应该保持一致的数据结构，方便服务端统一解析。  
```
{将制定好的结构写在这里}
```
## Mqtt Topic
Mqtt Topic规范为: 
```
/{position}/data|status|cmd|ws/{datatype}|gateway|node/{gatewaycode}/{nodecode}
```
例如上海电数据的字节数据如果是通过`04a3`网关下的`34ff`设备传上来的则应该是：
```
/shanghai/data/elec/gw04a3/nd34ff
```
byte是设备采集数据后，发布到的主题。conf则是设备订阅的主题，等待服务端下发配置，同时利用其hook监听设备在线状态。
如果采集设备本身是网关节点则`Topic`为：
```
/{position}/{type}/{byte|conf}/{gatewayid}
```
## 测试Mqtt连接
```yml
server: 120.76.136.124
port: 1883
web: http://120.76.136.124:18083
webusername: admin
webpassword: public
```
通过web界面可以查看客户端是否已经连接，选择websocket项订阅相关主题可以检测发布消息是否成功。  
也可以自己写两个客户端互相测试，参考[mqttdemo](https://github.com/sunwu51/mqttdemo)
## 接入流程
- 1.在web系统注册网关（gateway）和节点（node），获取相应的序列号。
- 2.网关按照自己和下属节点的序列号，订阅相应的Topic，并在规定的Topic上发布数据。
- 3.服务端解析数据部分负责解析
## 最后效果
- 设备可以采集数据，通过网关发送到Mqtt服务器。【注意数据格式和mqtt topic的规范】 
- 如果设备本身就可以入网，例如树莓派。则认为是网关设备，topic中`/{devid}`去掉即可。
- 后期网关需要监听相关的topic进行反向配置，可以先不考虑这种情况。
