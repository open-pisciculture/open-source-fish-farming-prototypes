# 1. Create the role below
aws iam create-role --role-name lambda-ex --assume-role-policy-document '{"Version": "2012-10-17","Statement": [{ "Effect": "Allow", "Principal": {"Service": "lambda.amazonaws.com"}, "Action": "sts:AssumeRole"}]}'

# 2. Create a directory and the script
mkdir iot-handler
cd iot-handler
nano lambda_function.py
pip install --target ./package requests # or any library, and repeat for each one
cd package

# 3. Add to .zip file
zip -r ../dep_pkg.zip .
cd ..
zip -g dep_pkg.zip lambda_function.py # update each time we want to change code

# CREATE/UPDATE THE LAMBDA
aws lambda create-function --function-name iot-handler --zip-file fileb://dep_pkg.zip --handler lambda_function.lambda_handler --runtime python3.8 --role arn:aws:iam::289667274164:role/aws-lambda-can-use-dynamo

# # Update with new code
# cd to path ~/Documents/Piscicultura/tests-Aws/aws-iot-handler/iot-handler
# First add to .zip file)
zip -g dep_pkg.zip lambda_function.py # update each time we want to change code
# Then run update command
aws lambda update-function-code --function-name iot-handler --zip-file fileb://dep_pkg.zip
