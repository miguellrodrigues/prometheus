#include <entityx.h>

#define ENTITYX_INITIAL_CAPACITY 10

// Funções auxiliares
const char* entityx_type_to_string(entityx_type_t type) {
    switch (type) {
        case ENTITY_TYPE_USER: return "USER";
        case ENTITY_TYPE_PRODUCT: return "PRODUCT";
        case ENTITY_TYPE_SENSOR: return "SENSOR";
        default: return "UNKNOWN";
    }
}

entityx_type_t entityx_string_to_type(const char* type_str) {
    if (strcmp(type_str, "USER") == 0) return ENTITY_TYPE_USER;
    if (strcmp(type_str, "PRODUCT") == 0) return ENTITY_TYPE_PRODUCT;
    if (strcmp(type_str, "SENSOR") == 0) return ENTITY_TYPE_SENSOR;
    return ENTITY_TYPE_UNKNOWN;
}

/**
 * @brief Cria um novo gerenciador de entidades
 * @param file_path Caminho do arquivo para persistência
 * @return entityx_t* Ponteiro para o gerenciador ou NULL em caso de erro
 */
entityx_t* entityx_create(const char* file_path) {
    entityx_t* manager = calloc(1, sizeof(entityx_t));
    if (!manager) {
        return NULL;
    }

    manager->entries = calloc(ENTITYX_INITIAL_CAPACITY, sizeof(entityx_entry_t*));
    if (!manager->entries) {
        free(manager);
        return NULL;
    }

    manager->capacity = ENTITYX_INITIAL_CAPACITY;
    manager->count = 0;
    manager->file_path = file_path;

    return manager;
}

/**
 * @brief Adiciona uma nova entidade ao gerenciador
 * @return entityx_status_t
 */
entityx_status_t entityx_add(entityx_t* manager, void* data, size_t size, entityx_type_t type,
                             void (*free_fn)(void*), int (*compare_fn)(const void*, const void*)) {
    if (!manager || !data || type <= ENTITY_TYPE_UNKNOWN || type >= ENTITY_TYPE_MAX) {
        return ENTITYX_INVALID_PARAM;
    }

    // Expandir array se necessário
    if (manager->count >= manager->capacity) {
        size_t new_capacity = manager->capacity * 2;
        entityx_entry_t** new_entries = realloc(manager->entries,
                                                new_capacity * sizeof(entityx_entry_t*));
        if (!new_entries) {
            return ENTITYX_MEMORY_ERROR;
        }
        manager->entries = new_entries;
        manager->capacity = new_capacity;
    }

    // Criar nova entrada
    entityx_entry_t* entry = calloc(1, sizeof(entityx_entry_t));
    if (!entry) {
        return ENTITYX_MEMORY_ERROR;
    }

    // Alocar e copiar dados
    entry->data = malloc(size);
    if (!entry->data) {
        free(entry);
        return ENTITYX_MEMORY_ERROR;
    }

    memcpy(entry->data, data, size);
    entry->size = size;
    entry->type = type;
    entry->free_fn = free_fn;
    entry->compare_fn = compare_fn;

    manager->entries[manager->count++] = entry;
    return ENTITYX_OK;
}

/**
 * @brief Remove uma entidade do gerenciador
 * @return entityx_status_t
 */
entityx_status_t entityx_remove(entityx_t* manager, entityx_entry_t* entry) {
    if (!manager || !entry) {
        return ENTITYX_INVALID_PARAM;
    }

    for (size_t i = 0; i < manager->count; i++) {
        if (manager->entries[i] == entry) {
            // Liberar memória da entidade
            if (entry->free_fn) {
                entry->free_fn(entry->data);
            } else {
                free(entry->data);
            }
            free(entry);

            // Reorganizar array
            memmove(&manager->entries[i],
                    &manager->entries[i + 1],
                    (manager->count - i - 1) * sizeof(entityx_entry_t*));

            manager->count--;
            return ENTITYX_OK;
        }
    }

    return ENTITYX_NOT_FOUND;
}

/**
 * @brief Busca entidades por um critério específico
 * @return entityx_status_t
 */
entityx_status_t entityx_find(entityx_t* manager, const void* key,
                              int (*compare_fn)(const void*, const void*),
                              entityx_entry_t** results, size_t max_results,
                              size_t* found_count) {
    if (!manager || !key || !compare_fn || !results || !found_count) {
        return ENTITYX_INVALID_PARAM;
    }

    *found_count = 0;

    for (size_t i = 0; i < manager->count && *found_count < max_results; i++) {
        if (compare_fn(manager->entries[i]->data, key) == 0) {
            results[(*found_count)++] = manager->entries[i];
        }
    }

    return *found_count > 0 ? ENTITYX_OK : ENTITYX_NOT_FOUND;
}

/**
 * @brief Busca entidades por tipo
 * @return entityx_status_t
 */
entityx_status_t entityx_find_by_type(entityx_t* manager, entityx_type_t type,
                                      entityx_entry_t** results, size_t max_results,
                                      size_t* found_count) {
    if (!manager || !results || !found_count ||
        type <= ENTITY_TYPE_UNKNOWN || type >= ENTITY_TYPE_MAX) {
        return ENTITYX_INVALID_PARAM;
    }

    *found_count = 0;

    for (size_t i = 0; i < manager->count && *found_count < max_results; i++) {
        if (manager->entries[i]->type == type) {
            results[(*found_count)++] = manager->entries[i];
        }
    }

    return *found_count > 0 ? ENTITYX_OK : ENTITYX_NOT_FOUND;
}

/**
 * @brief Salva todas as entidades em arquivo
 * @return entityx_status_t
 */
entityx_status_t entityx_save(entityx_t* manager) {
    if (!manager || !manager->file_path) {
        return ENTITYX_INVALID_PARAM;
    }

    FILE* file = fopen(manager->file_path, "wb");
    if (!file) {
        return ENTITYX_FILE_ERROR;
    }

    // Salvar número de entidades
    fwrite(&manager->count, sizeof(size_t), 1, file);

    // Salvar cada entidade
    for (size_t i = 0; i < manager->count; i++) {
        entityx_entry_t* entry = manager->entries[i];

        // Salvar tipo e tamanho
        fwrite(&entry->type, sizeof(entityx_type_t), 1, file);
        fwrite(&entry->size, sizeof(size_t), 1, file);

        // Salvar dados
        fwrite(entry->data, 1, entry->size, file);
    }

    fclose(file);
    return ENTITYX_OK;
}

/**
 * @brief Carrega entidades do arquivo
 * @return entityx_status_t
 */
entityx_status_t entityx_load(entityx_t* manager,
                              void* (*create_fn)(entityx_type_t, const void*, size_t),
                              void (*free_fn)(void*),
                              int (*compare_fn)(const void*, const void*)) {
    if (!manager || !manager->file_path || !create_fn) {
        return ENTITYX_INVALID_PARAM;
    }

    FILE* file = fopen(manager->file_path, "rb");
    if (!file) {
        return ENTITYX_FILE_ERROR;
    }

    // Ler número de entidades
    size_t count;
    fread(&count, sizeof(size_t), 1, file);

    for (size_t i = 0; i < count; i++) {
        entityx_type_t type;
        size_t size;

        // Ler tipo e tamanho
        fread(&type, sizeof(entityx_type_t), 1, file);
        fread(&size, sizeof(size_t), 1, file);

        // Ler dados
        void* temp_data = malloc(size);
        if (!temp_data) {
            fclose(file);
            return ENTITYX_MEMORY_ERROR;
        }

        fread(temp_data, 1, size, file);

        // Criar entidade usando a função de criação fornecida
        void* data = create_fn(type, temp_data, size);
        free(temp_data);

        if (!data) {
            fclose(file);
            return ENTITYX_MEMORY_ERROR;
        }

        // Adicionar entidade ao gerenciador
        entityx_status_t status = entityx_add(manager, data, size, type, free_fn, compare_fn);
        free(data);

        if (status != ENTITYX_OK) {
            fclose(file);
            return status;
        }
    }

    fclose(file);
    return ENTITYX_OK;
}

/**
 * @brief Libera toda a memória do gerenciador
 */
void entityx_destroy(entityx_t* manager) {
    if (!manager) {
        return;
    }

    for (size_t i = 0; i < manager->count; i++) {
        entityx_entry_t* entry = manager->entries[i];
        if (entry->free_fn) {
            entry->free_fn(entry->data);
        } else {
            free(entry->data);
        }
        free(entry);
    }

    free(manager->entries);
    free(manager);
}

/**
 * @brief Obtém o número de entidades no gerenciador
 */
size_t entityx_count(entityx_t* manager) {
    return manager ? manager->count : 0;
}

/**
 * @brief Obtém a capacidade atual do gerenciador
 */
size_t entityx_capacity(entityx_t* manager) {
    return manager ? manager->capacity : 0;
}
