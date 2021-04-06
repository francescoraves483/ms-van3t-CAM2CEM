#include "simpleCAMSenderCEM-helper.h"

#include "ns3/simpleCAMSender-gps-tc-cem.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

simpleCAMSenderCEMHelper::simpleCAMSenderCEMHelper ()
{
  m_factory.SetTypeId (simpleCAMSenderCEM::GetTypeId ());
}


void 
simpleCAMSenderCEMHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
simpleCAMSenderCEMHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
simpleCAMSenderCEMHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
simpleCAMSenderCEMHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
simpleCAMSenderCEMHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<simpleCAMSenderCEM> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
