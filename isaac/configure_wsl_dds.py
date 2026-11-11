"""Add an MTU-safe UDP transport to the existing user's Fast DDS peer profile."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

profile=Path.home()/'.ros'/'fastdds.xml'
backup=profile.with_name('fastdds.xml.bak_camera_20261105')
if not backup.exists():
    shutil.copy2(profile,backup)
ns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'
ET.register_namespace('',ns)
def tag(name):return '{'+ns+'}'+name
root=ET.parse(profile).getroot()
descriptors=root.find(tag('transport_descriptors'))
if descriptors is None:descriptors=ET.SubElement(root,tag('transport_descriptors'))
root.remove(descriptors)
root.insert(0,descriptors)  # Fast DDS resolves transport IDs while reading participants.
for descriptor in list(descriptors):
    if descriptor.findtext(tag('transport_id'))=='welding_udp':descriptors.remove(descriptor)
descriptor=ET.SubElement(descriptors,tag('transport_descriptor'))
for name,value in {'transport_id':'welding_udp','type':'UDPv4','maxMessageSize':'1400',
                   'sendBufferSize':'16777216','receiveBufferSize':'16777216'}.items():
    ET.SubElement(descriptor,tag(name)).text=value
for rtps in root.findall('.//'+tag('rtps')):
    for name in ('userTransports','useBuiltinTransports'):
        old=rtps.find(tag(name))
        if old is not None:rtps.remove(old)
    transports=ET.SubElement(rtps,tag('userTransports'))
    ET.SubElement(transports,tag('transport_id')).text='welding_udp'
    ET.SubElement(rtps,tag('useBuiltinTransports')).text='false'
tree=ET.ElementTree(root)
ET.indent(tree)
tree.write(profile,encoding='utf-8',xml_declaration=True)
print('Updated',profile,'; backup:',backup)
