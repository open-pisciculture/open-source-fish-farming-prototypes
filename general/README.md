# General

Here supporting information common to various prototypes can be found.

- Setting up the LoRaWAN Gateway with The Things Network
- Creating devices in The Things Network
- Removing ST-Link Programmer and Solder Bridges from Nucleo board
- Programming the NUCLEO board with the removed programmer
- Deployment of AWS services

## Setting up the LoRaWAN Gateway with The Things Network
The Gateway used for this prototype is [this one](https://www.sparkfun.com/products/16447). First, make sure to follow the Getting Started Guide available under the Documents tab in the [Sparkfun website](https://cdn.sparkfun.com/assets/8/3/6/d/4/Getting_Started_Guide_for_RAK7244_V1.0.pdf).

## Creating devices in The Things Network

## Removing ST-Link Programmer and Solder Bridges from Nucleo board

## Programming the NUCLEO board with the removed programmer

## Deployment of AWS services

![AWS services](images/aws_diagram.png)

First, make sure to follow the [Deployment Guide](https://www.thethingsindustries.com/docs/integrations/cloud-integrations/aws-iot/deployment-guide/) from The Things Network to deploy the AWS IoT integration for The Things Stack. This will create a CloudFormation stack in you AWS account containing several AWS resources necessary to integrate The Things Networks with your AWS account (follow mentioned documentation above to find out more about them).

Second, in this repository we developed another CloudFormation template including the following resources (*Important note:* there are many ways to handle TTN received data, this way, including a Lambda function, is recommended since it gives the developer more flexibility):

- DynamoDB table where data is sent.
- Lambda function.
- IoT Rule that queries/filters data received via MQTT topic at AWS IoT Core.
- Associated IAM Permissions.

Simply go to the AWS Console > CloudFormation > Create stack > With new resources > Upload a template file > Upload template `open-source-fish-farming-prototypes/general/cloud/iot_lambda_dynamo.template.yaml` > Choose any stack name > Leave everything as default > Create stack. This template has a parameter (`SQLQueryForIoTRule`) which can be adapted to the developer's needs, there are a couple more examples in `open-source-fish-farming-prototypes/general/cloud/queries_aws_iot_rule.txt`.

When appropriately following these steps, you should end up with a `CREATE_COMPLETE` status, you can double check this in the AWS Console > CloudFormation > Search bar '<your stack name>'.

<p align="middle">
  <img src="images/stack_status.png" />
</p>


