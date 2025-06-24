// clang-format off
// Visual Studio Community keyboard shortcuts:  Format Document (file):  Ctrl+K, Ctrl+D     Format Selected Code:  Ctrl+K, Ctrl+F

/*  Created by este este - 28-MAR-2025
      @ Lists all partitions on the ESP32 chip.
      @ If the default 'nsv' partition is found then any namespaces in that partition are listed.
      @ For each namespace in the default 'nsv' partition, the key-value pairs are listed.
*/

// To check partitions
#include "esp_partition.h"
#include "esp_log.h"

// To check namespaces in the nvs partition
#include "nvs_flash.h"
#include "nvs.h"

// To manage unique namespaces
#include <set>

/*
The ESP32 Non-Volatile Storage (NVS) system supports the following value types for storing data
via the nvs_type_t enum which has the following values:
  NVS_TYPE_U8: Unsigned 8-bit integer.
  NVS_TYPE_I8: Signed 8-bit integer.
  NVS_TYPE_U16: Unsigned 16-bit integer.
  NVS_TYPE_I16: Signed 16-bit integer.
  NVS_TYPE_U32: Unsigned 32-bit integer.
  NVS_TYPE_I32: Signed 32-bit integer.
  NVS_TYPE_U64: Unsigned 64-bit integer.
  NVS_TYPE_I64: Signed 64-bit integer.
  NVS_TYPE_STR: Null-terminated string.
  NVS_TYPE_BLOB: Binary large object (arbitrary binary data).
  NVS_TYPE_ANY: A special type used during iteration to indicate that any type of entry is acceptable.
*/

bool listEspPartitions()
{
  /* <<<<< FIND ALL PARTITIONS >>>>> */

  bool defaultNvsPartitionFoundQ = false;

  // Tag for logging
  const char *TAG = "PARTITIONS";

  // Iterator to find all partitions
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);

  if ( it == NULL )
    {
      ESP_LOGI(TAG, "No partitions found");
      Serial.println("No partitions found.");
      return defaultNvsPartitionFoundQ;
    }

  Serial.println("\nLocating ALL Partitions...\n");

  // Iterate through all partitions
  while ( it != NULL )
    {
      const esp_partition_t *partition = esp_partition_get(it);

      // Print partition details
      ESP_LOGI(TAG, "Found Partition:");
      ESP_LOGI(TAG, "Label: %s, Address: 0x%X, Size: %d bytes, Type: %d, Subtype: %d", partition->label, partition->address, partition->size, partition->type, partition->subtype);

      Serial.printf("Found Partition:\n");
      Serial.printf("\tLabel: %s\n", partition->label);

      if ( partition->subtype == ESP_PARTITION_SUBTYPE_DATA_NVS )
        {
          Serial.printf("     Partition labeled '%s' is an NVS partition.\n", partition->label);
          if ( String(partition->label) == "nvs" )
            {
              defaultNvsPartitionFoundQ = true;
            }
        }

      Serial.printf("\tAddress: 0x%X\n", partition->address);
      Serial.printf("\tSize: %d bytes\n", partition->size);
      Serial.printf("\tType: %d\n", partition->type);
      Serial.printf("\tSubtype: %d\n", partition->subtype);
      Serial.printf("");

      // Move to the next partition
      it = esp_partition_next(it);
    }

  // Release the iterator
  esp_partition_iterator_release(it);

  if ( !defaultNvsPartitionFoundQ )
    {
      Serial.printf("\nDefault 'nvs' partition was NOT found so exiting.\n");
      return defaultNvsPartitionFoundQ;
    }
  else
    {
      defaultNvsPartitionFoundQ = true;
      Serial.printf("\nDefault 'nvs' partition WAS found so continuing.\n");
      return defaultNvsPartitionFoundQ;
    }
}

bool listUniqueNvsNamespaces()
{
  bool defaultNvsNameSpaceFoundQ = false;

  // Initialize default NVS partition 'nvs'
  esp_err_t err = nvs_flash_init();  // only works for the default partition.
  if ( err != ESP_OK )
    {
      Serial.printf("Failed to initialize NVS partition with default name of 'nvs': %s\n", esp_err_to_name(err));
      return defaultNvsNameSpaceFoundQ;
    }

  // Create an iterator for entries in NVS
  nvs_iterator_t it = nvs_entry_find(NVS_DEFAULT_PART_NAME, NULL, NVS_TYPE_ANY);
  if ( it == NULL )
    {
      Serial.println("\nNo namespaces found.");
      return defaultNvsNameSpaceFoundQ;
    }

  std::set<String> uniqueNamespaces;  // To store unique namespace names
  Serial.println("\nNamespaces in the default 'nvs' partition:");
  while ( it != NULL )
    {
      defaultNvsNameSpaceFoundQ = true;
      nvs_entry_info_t info;
      nvs_entry_info(it, &info);

      // Add unique namespaces to the set
      if ( uniqueNamespaces.find(String(info.namespace_name)) == uniqueNamespaces.end() )
        {
          uniqueNamespaces.insert(String(info.namespace_name));
          Serial.printf("- Namespace: %s\n", info.namespace_name);
        }

      it = nvs_entry_next(it);
    }
  return defaultNvsNameSpaceFoundQ;
}

bool listNamespacesWithKeysAndValues(const char *partition_label)
{
  bool             successQ = false;

  std::set<String> uniqueNamespaces;  // Store unique namespace names
  nvs_iterator_t   it = nvs_entry_find(partition_label, NULL, NVS_TYPE_ANY);
  if ( it == NULL )
    {
      Serial.printf("\nNo namespaces found in partition '%s'\n", partition_label);
      return successQ;
    }

  while ( it != NULL )
    {
      successQ = true;
      nvs_entry_info_t info;
      nvs_entry_info(it, &info);

      // Add to unique set and process if not already listed
      if ( uniqueNamespaces.find(String(info.namespace_name)) == uniqueNamespaces.end() )
        {
          uniqueNamespaces.insert(String(info.namespace_name));
          Serial.printf("\nNamespace: %s\n", info.namespace_name);
          listKeysAndValues(partition_label, info.namespace_name);
        }
      it = nvs_entry_next(it);
    }
  return successQ;
}

bool listKeysAndValues(const char *partition_label, const char *namespace_name)
{
  bool         successQ = false;
  nvs_handle_t handle;
  esp_err_t    err;

  // Open the namespace in the specified partition
  err = nvs_open_from_partition(partition_label, namespace_name, NVS_READONLY, &handle);
  if ( err != ESP_OK )
    {
      Serial.printf("\nFailed to open namespace '%s' in partition '%s'\n", namespace_name, partition_label);
      return successQ;
    }

  Serial.printf("Keys and values in namespace '%s':\n", namespace_name);

  nvs_iterator_t it = nvs_entry_find(partition_label, namespace_name, NVS_TYPE_ANY);
  while ( it != NULL )
    {
      successQ = true;
      nvs_entry_info_t info;
      nvs_entry_info(it, &info);

      Serial.printf("- Key: %s\n", info.key);

      // Handle all supported value types
      switch ( info.type )
        {
          case NVS_TYPE_U8:
            {
              uint8_t value;
              if ( nvs_get_u8(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (uint8): %u\n", value);
                }
              break;
            }
          case NVS_TYPE_I8:
            {
              int8_t value;
              if ( nvs_get_i8(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (int8): %d\n", value);
                }
              break;
            }
          case NVS_TYPE_U16:
            {
              uint16_t value;
              if ( nvs_get_u16(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (uint16): %u\n", value);
                }
              break;
            }
          case NVS_TYPE_I16:
            {
              int16_t value;
              if ( nvs_get_i16(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (int16): %d\n", value);
                }
              break;
            }
          case NVS_TYPE_U32:
            {
              uint32_t value;
              if ( nvs_get_u32(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (uint32): %u\n", value);
                }
              break;
            }
          case NVS_TYPE_I32:
            {
              int32_t value;
              if ( nvs_get_i32(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (int32): %d\n", value);
                }
              break;
            }
          case NVS_TYPE_U64:
            {
              uint64_t value;
              if ( nvs_get_u64(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (uint64): %llu\n", value);
                }
              break;
            }
          case NVS_TYPE_I64:
            {
              int64_t value;
              if ( nvs_get_i64(handle, info.key, &value) == ESP_OK )
                {
                  Serial.printf("  Value (int64): %lld\n", value);
                }
              break;
            }
          case NVS_TYPE_STR:
            {
              size_t required_size = 0;
              nvs_get_str(handle, info.key, NULL, &required_size);
              char *value = (char *)malloc(required_size);
              if ( value != NULL && nvs_get_str(handle, info.key, value, &required_size) == ESP_OK )
                {
                  Serial.printf("  Value (string): %s\n", value);
                }
              free(value);
              break;
            }
          case NVS_TYPE_BLOB:
            {
              size_t required_size = 0;
              nvs_get_blob(handle, info.key, NULL, &required_size);
              uint8_t *blob = (uint8_t *)malloc(required_size);
              if ( blob != NULL && nvs_get_blob(handle, info.key, blob, &required_size) == ESP_OK )
                {
                  Serial.printf("  Value (blob): [size: %d bytes]\n", required_size);
                }
              free(blob);
              break;
            }
          default:
            Serial.printf("  Unsupported type\n");
            break;
        }
      it = nvs_entry_next(it);
    }
  nvs_close(handle);
  return successQ;
}

void setup()
{
  Serial.begin(115200);

  if ( !listEspPartitions() )
    {
      return;
    }

  if ( !listUniqueNvsNamespaces() )
    {
      return;
    }

  if ( !listNamespacesWithKeysAndValues("nvs") )
    {
      return;
    }
}

void loop()
{
  // Empty loop
}
