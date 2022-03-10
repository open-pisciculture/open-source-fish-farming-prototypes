# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Purpose
Shows how to implement an AWS Lambda function that handles input from direct
invocation.
"""

import boto3
import json
from decimal import Decimal
import os

DYNAMODB_TABLE_NAME = os.environ['DYNAMODB_TABLE_NAME']

# Get the service resource.
dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table(DYNAMODB_TABLE_NAME)

def lambda_handler(event, context):
    ev_to_save = json.loads(json.dumps(event), parse_float=Decimal)
    print(f"ev_to_save: {ev_to_save}")
    table.put_item(Item=ev_to_save)
